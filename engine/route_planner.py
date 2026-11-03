"""Net ordering, rip-up recovery, and route-planning orchestration."""

import json
import math
import os
import time

from shapely.geometry import Point

from router_constants import *  # noqa: F403
from search_model import Env, Pad
from path_search import (
    _emit_path, _route_pair_parallel, _route_chained_net,
)
from route_refinement import postprocess_135_routes


def _stitch_pad(env, net, pad, zone_layer):
    """Place one via + stub so an SMD pad reaches its plane on zone_layer.
    Assumes env.obst.rebuild_for_net(net) was already done."""
    base = pad.rmax + env.via_r + env.clearance + SAFETY + 0.05
    for ring in (base, base * 1.4, base * 1.9, base * 2.5, base * 3.2):
        for ang in range(0, 360, 15):
            a = math.radians(ang)
            p = (pad.x + ring * math.cos(a), pad.y + ring * math.sin(a))
            if not env.keep_via.contains(Point(p)):
                continue
            if not env.hole_ok(p):
                continue
            if not env.obst.wire_clear(pad.layer, (pad.x, pad.y), p):
                continue
            if not env.obst.via_clear(env.copper, p[0], p[1], env.via_r):
                continue
            if not env.inside_filled_zone(net, zone_layer, p):
                continue
            env.emit_track(net, pad.layer, (pad.x, pad.y), p)
            env.emit_via(net, p)
            return True
    return False


def _span(multi, n):
    ps = multi[n]
    xs = [p.x for p in ps]
    ys = [p.y for p in ps]
    return (max(xs) - min(xs)) + (max(ys) - min(ys))


def _riproute(env, target_multi, repair_multi, protected_nets,
              failures, stats, verbose, budget=RIPUP_BUDGET,
              board_deadline=None, blocker_hints=None):
    """Targeted rip-up-and-reroute: the one-pass order can wall a net in even
    when every net fits alone (isolation test says 44/44). For each failed
    net, use observed search collisions to identify routed blockers, rip a small set,
    re-route the failed net, then re-route everything ripped.

    ``target_multi`` contains the nets selected by this routing stage, while
    ``repair_multi`` contains every net for which complete original pad
    topology is available. This distinction lets an escape pass promote a
    fixed pre-route safely instead of indexing a target-only mapping and
    raising KeyError. Every edit is transactional: ripped victims are always
    restored, and a transaction is committed only when the total open-
    connection score improves. A partially restored victim is permitted when
    the board-wide open-connection count falls. Via count is recorded as a
    quality metric but never blocks a connectivity improvement.
    """
    blocker_hints = blocker_hints or {}
    deadline = time.monotonic() + float(budget)
    if board_deadline is not None:
        deadline = min(deadline, board_deadline)
    fixed = 0
    improved_commits = 0
    diagnostics = {}
    promoted_attempts = promoted_commits = rollbacks = combo_attempts = 0
    added_via_commits = added_vias = 0
    transaction_errors = []
    fixed_blocker_nets = set()
    initial_vias = sum(1 for item in env.items
                       if item.get("type") == "via")
    per_net_budgets = []
    nets_considered = 0
    nets_with_blockers = 0
    nets_budget_exhausted = 0

    def record(net, reason, blockers=(), fixed_blockers=(), errors=(),
               observed_blockers=None):
        diagnostics[net] = {
            "net": net,
            "reason": reason,
            "blockers": sorted(set(blockers)),
            "fixed_blockers": sorted(set(fixed_blockers)),
            "observed_blocker_hits": dict(sorted(
                (observed_blockers or {}).items(),
                key=lambda pair: (-pair[1], pair[0]))[:12]),
            "reroute_attempted": bool(blockers) or reason in {
                "reroute_transaction_rejected", "reroute_transaction_error",
            },
            "errors": list(errors),
        }

    def connectivity_score(failure_map):
        """Rank a tentative transaction by remaining open connections.

        The per-net router can leave a partially useful tree even when it
        cannot finish every branch. Requiring every ripped victim to reconnect
        completely discards those useful improvements. Count open branches
        first, then the number of affected nets, matching the board-level
        completion-first objective.
        """
        return (sum(len(items) for items in failure_map.values()),
                len(failure_map))

    # fewest-unrouted first: cheap wins early, and a still-failed net's own
    # partial copper (emitted before its last pad gave up) stays put
    ordered_failures = sorted(list(failures), key=lambda n: len(failures[n]))
    for index, net in enumerate(ordered_failures):
        now = time.monotonic()
        nets_left = len(ordered_failures) - index
        remaining_budget = max(0.0, deadline - now)
        if remaining_budget <= 0.0:
            nets_budget_exhausted += nets_left
            record(net, "ripup_budget_exhausted")
            for other in ordered_failures[index + 1:]:
                record(other, "ripup_budget_exhausted")
            break
        allocated = min(RIPUP_PER_NET_BUDGET,
                        remaining_budget / max(1, nets_left))
        allocated = max(min(RIPUP_PER_NET_BUDGET,
                            MIN_RIPUP_PER_NET_BUDGET), allocated)
        net_deadline = min(deadline, now + allocated)
        per_net_budgets.append(max(0.0, net_deadline - now))
        nets_considered += 1
        if now > net_deadline:
            nets_budget_exhausted += 1
            record(net, "ripup_budget_exhausted")
            continue
        pads = target_multi.get(net)
        if not pads:
            record(net, "missing_target_topology")
            continue
        env.use_net_rule(net)
        crossing = {}
        fixed_crossing = {}
        routed_nets = {it.get("net") for it in env.items
                       if it.get("type") in {"track", "via"}}
        observed = blocker_hints.get(net, {})
        for n2, hit_count in observed.items():
            if n2 == net:
                continue
            if n2 not in routed_nets:
                fixed_crossing[n2] = fixed_crossing.get(n2, 0) + hit_count
                continue
            victim_pads = repair_multi.get(n2)
            if n2 in protected_nets:
                fixed_crossing[n2] = fixed_crossing.get(n2, 0) + hit_count
                fixed_blocker_nets.add(n2)
            elif victim_pads and len(victim_pads) >= 2:
                crossing[n2] = crossing.get(n2, 0) + hit_count
            else:
                fixed_crossing[n2] = fixed_crossing.get(n2, 0) + hit_count
                fixed_blocker_nets.add(n2)
        if not crossing:
            reason = ("blocked_by_fixed_route" if fixed_crossing else
                      "no_search_blocker_observed" if not observed else
                      "no_reroutable_search_blocker")
            record(net, reason, fixed_blockers=fixed_crossing,
                   observed_blockers=observed)
            continue
        nets_with_blockers += 1
        # Test the strongest observed blocker alone, then together with the
        # next strongest if one blocker is insufficient. This bounds retries
        # while basing every transaction on actual rejected path geometry.
        cands = sorted(crossing, key=lambda n: (-crossing[n], n))[:4]
        combos = [(cands[0],)]
        if len(cands) > 1:
            combos.append(tuple(cands[:2]))
        attempted = []
        budget_exhausted = False
        committed = False
        for combo in combos[:MAX_RIPUP_COMBOS_PER_NET]:
            if time.monotonic() > net_deadline:
                budget_exhausted = True
                break
            combo_attempts += 1
            attempted.extend(combo)
            promoted = [victim for victim in combo
                        if victim in env.preitem_nets
                        and victim not in target_multi]
            if promoted:
                promoted_attempts += 1
            snap = env.snapshot()
            before_failure_score = connectivity_score(failures)
            before_vias = sum(1 for item in env.items
                              if item.get("type") == "via")
            try:
                for victim in combo:
                    env.rip_net(victim)
                env.rip_net(net)  # pass-2 partial routes: full re-emit
                env.use_net_rule(net)
                env.obst.rebuild_for_net(net)
                fails2, _p = _route_chained_net(
                    env, net, pads, deadline=net_deadline)
                candidate_failures = dict(failures)
                if fails2:
                    candidate_failures[net] = fails2
                else:
                    candidate_failures.pop(net, None)

                # Always restore every ripped victim, even when the target
                # net remains partial. A transaction may still improve the
                # total number of open branches and is committed only when
                # the board-wide completion score strictly improves.
                reroute_fails = []
                for victim in combo:
                    victim_pads = repair_multi.get(victim)
                    if not victim_pads:
                        vf = ["missing complete topology"]
                    else:
                        env.use_net_rule(victim)
                        env.obst.rebuild_for_net(victim)
                        vf, _vp = _route_chained_net(
                            env, victim, victim_pads,
                            deadline=net_deadline)
                    if vf:
                        candidate_failures[victim] = vf
                        reroute_fails.append((victim, vf))
                    else:
                        candidate_failures.pop(victim, None)

                after_failure_score = connectivity_score(candidate_failures)
                if after_failure_score < before_failure_score:
                    after_vias = sum(1 for item in env.items
                                     if item.get("type") == "via")
                    via_delta = max(0, after_vias - before_vias)
                    failures.clear()
                    failures.update(candidate_failures)
                    improved_commits += 1
                    if net not in failures:
                        fixed += 1
                    if via_delta:
                        added_via_commits += 1
                        added_vias += via_delta
                    if promoted and net not in failures:
                        promoted_commits += 1
                    if net in failures:
                        diagnostics[net] = {
                            "net": net,
                            "reason": "partial_transaction_committed",
                            "blockers": sorted(set(combo)),
                            "fixed_blockers": sorted(set(fixed_crossing)),
                            "reroute_attempted": True,
                            "errors": [],
                            "before_open_connections": before_failure_score[0],
                            "after_open_connections": after_failure_score[0],
                            "victims_still_open": sorted(
                                victim for victim, _ in reroute_fails),
                        }
                    else:
                        diagnostics.pop(net, None)
                    committed = True
                    if verbose:
                        print(f"  [LE] rip-up improved connectivity "
                              f"{before_failure_score[0]} -> "
                              f"{after_failure_score[0]} open connections "
                              f"(target {net}; ripped {'+'.join(combo)})")
                    break        # next failed net
                rollbacks += 1
                env.restore(snap)  # incomplete target/victim: exact rollback
            except Exception as exc:
                # A reroute may fail after it has removed or emitted copper.
                # Restore the exact pre-transaction state before trying any
                # other blocker combination; never leak a partial mutation.
                rollbacks += 1
                env.restore(snap)
                transaction_errors.append(
                    f"{net} via {'+'.join(combo)}: "
                    f"{type(exc).__name__}: {exc}")
                continue
        if net in failures and not committed:
            if budget_exhausted or time.monotonic() >= net_deadline:
                nets_budget_exhausted += 1
            net_errors = [error for error in transaction_errors
                          if error.startswith(f"{net} via ")]
            reason = ("reroute_transaction_error" if net_errors else
                      "ripup_budget_exhausted" if budget_exhausted else
                      "reroute_transaction_rejected")
            record(net, reason, blockers=attempted,
                   fixed_blockers=fixed_crossing, errors=net_errors,
                   observed_blockers=observed)
    stats["ripup_fixed"] = fixed
    stats["ripup_budget_seconds"] = float(budget)
    stats["ripup_per_net_budget_max_seconds"] = RIPUP_PER_NET_BUDGET
    stats["ripup_per_net_budget_min_seconds"] = MIN_RIPUP_PER_NET_BUDGET
    stats["ripup_nets_considered"] = nets_considered
    stats["ripup_nets_with_blockers"] = nets_with_blockers
    stats["ripup_nets_budget_exhausted"] = nets_budget_exhausted
    stats["ripup_per_net_budget_max_used_seconds"] = max(
        per_net_budgets, default=0.0)
    stats["ripup_promoted_attempts"] = promoted_attempts
    stats["ripup_promoted_commits"] = promoted_commits
    stats["ripup_improved_commits"] = improved_commits
    stats["ripup_rollbacks"] = rollbacks
    # Retain the old field for sidecar compatibility; completion-first mode
    # never rejects a fully connected transaction because of its via count.
    stats["ripup_via_rejections"] = 0
    stats["ripup_added_via_commits"] = added_via_commits
    stats["ripup_added_vias"] = added_vias
    stats["ripup_combo_attempts"] = combo_attempts
    stats["ripup_transaction_errors"] = len(transaction_errors)
    stats["ripup_fixed_blockers_skipped"] = len(fixed_blocker_nets)
    stats["blocker_recovery_source"] = "observed_search_collisions"
    stats["blocker_hint_net_count"] = sum(bool(v) for v in blocker_hints.values())
    stats["blocker_hint_observations"] = sum(
        sum(v.values()) for v in blocker_hints.values())
    stats["ripup_initial_vias"] = initial_vias
    stats["ripup_final_vias"] = sum(
        1 for item in env.items if item.get("type") == "via")
    stats["blocked_routes"] = [diagnostics[n] for n in sorted(diagnostics)]
    return fixed


def _pairs(seq):
    for i in range(len(seq)):
        for j in range(i + 1, len(seq)):
            yield (seq[i], seq[j])


def _attempt(geo, via_cost, multi, zone_map, order, verbose, attempt_no,
             repair_multi=None, protected_nets=None,
             ripup_budget=RIPUP_BUDGET, deadline=None,
             route_deadline=None,
             allow_ripup=True):
    """One full routing attempt over `order` (list of trace-net names).
    Returns (items, failures, stats). Zone stitching runs first regardless."""
    env = Env(geo, via_cost)
    repair_multi = repair_multi or multi
    protected_nets = set(protected_nets or zone_map)
    for pad in env.pads:
        for layer in pad.accessible(env.copper):
            env.obst.add(layer, pad.rect(), pad.net,
                         pad.local_clearance)

    stats = {"stitched": 0, "through_pads": 0, "pairs": 0,
             "zone_nets": 0, "nets": len(multi), "attempt": attempt_no}
    failures = {}
    blocker_hints = {}

    # Zone nets are stitched after signal routes so their via fences cannot
    # consume pad-escape corridors before ordinary nets get a route.
    zone_nets = {n: zone_map[n] for n in multi if n in zone_map}
    stats["zone_nets"] = len(zone_nets)
    # -- pass 1: trace-routed nets in `order` ----------------------------------
    for net in order:
        if route_deadline is not None and time.monotonic() >= route_deadline:
            failures[net] = [
                f"{p.ref}@{p.x:.1f},{p.y:.1f} (route_time_budget_exhausted)"
                for p in multi[net]]
            continue
        env.use_net_rule(net)
        env.obst.rebuild_for_net(net)
        pads = multi[net]
        env.obst.blocker_hits = {}
        env.obst.capture_blockers = True
        try:
            fails, pairs = _route_chained_net(
                env, net, pads, deadline=route_deadline)
        finally:
            env.obst.capture_blockers = False
        stats["pairs"] += pairs
        if fails:
            failures[net] = fails
            blocker_hints[net] = dict(env.obst.blocker_hits)
        if verbose:
            tag = "OK  " if net not in failures else "FAIL"
            print(f"  [LE] net {net}: {len(pads)} pads {tag}")

    # Recover blocked signals before adding plane stitch vias; this keeps
    # zone access hardware from becoming a new fixed blocker for signal nets.
    if failures and allow_ripup:
        rip_snapshot = env.snapshot()
        failure_snapshot = {net: list(fails)
                            for net, fails in failures.items()}
        try:
            _riproute(env, multi, repair_multi, protected_nets,
                      failures, stats, verbose, budget=ripup_budget,
                      board_deadline=deadline,
                      blocker_hints=blocker_hints)
        except Exception as exc:
            import traceback
            env.restore(rip_snapshot)
            failures.clear()
            failures.update(failure_snapshot)
            stats["ripup_pass_error"] = f"{type(exc).__name__}: {exc}"
            traceback.print_exc()
            print(f"  [LE] rip-up pass aborted ({exc}) — keeping pass-1 result")

    # -- pass 2: zone stitching and fallback traces -----------------------------
    for net, zone_layer in sorted(zone_nets.items(),
                                  key=lambda kv: -len(multi[kv[0]])):
        if route_deadline is not None and time.monotonic() >= route_deadline:
            failures[net] = ["route_time_budget_exhausted"]
            continue
        env.use_net_rule(net)
        env.obst.rebuild_for_net(net)
        failed_stitches = []
        pads = multi[net]
        for pad in pads:
            if pad.through:
                stats["through_pads"] += 1
            elif pad.layer == zone_layer:
                continue
            elif _stitch_pad(env, net, pad, zone_layer):
                stats["stitched"] += 1
            else:
                failed_stitches.append(pad)
        if failed_stitches:
            good_pads = [p for p in pads if p not in failed_stitches] or [pads[0]]
            net_deadline = time.monotonic() + NET_TIME_BUDGET
            if route_deadline is not None:
                net_deadline = min(net_deadline, route_deadline)
            fails = []
            for pad in failed_stitches:
                anchors = sorted(
                    good_pads,
                    key=lambda c: math.hypot(c.x - pad.x, c.y - pad.y))
                path = _route_pair_parallel(
                    env, net, anchors[:MAX_PARALLEL_PATHS], pad,
                    net_deadline)
                if path is None:
                    fails.append(f"{pad.ref}@{pad.x:.1f},{pad.y:.1f}")
                else:
                    _emit_path(env, net, path)
                    stats["pairs"] += 1
            if fails:
                failures[net] = fails

    stats.setdefault("blocked_routes", [])
    stats.setdefault("ripup_transaction_errors", 0)

    stats["vias"] = sum(1 for i in env.items if i["type"] == "via")
    stats["wirelength"] = round(sum(
        math.hypot(i["b"][0] - i["a"][0], i["b"][1] - i["a"][1])
        for i in env.items if i.get("type") == "track"), 4)
    stats.update(env.search_stats)
    return env.items, failures, stats


def route_geometry(geo, via_cost=2.0, verbose=True, progress_json=None,
                   deadline=None):
    """Pure-geometry routing: geo dict in, {items, failures, stats} out.

    Route all nets once, then give failed nets bounded blocker-aware recovery.
    If failures remain, try at most two generic order variants: failed nets
    first, then longest-span first. Keep only the best result. These rules are
    board-independent; time and retry limits apply uniformly.

    progress_json: after EVERY attempt the current best is checkpointed
    there, so a geometry-stage crash mid-retry still leaves a usable board
    (the shapely subprocess has died silently twice on dense boards)."""
    nets = {}
    for d in geo["pads"]:
        pad = Pad(d)
        nets.setdefault(pad.net, []).append(pad)
    # Escape mode: only the audit-selected pads (zone-pocketed pads plus a
    # connected anchor each) get routed; every pad still lands in the
    # obstacle model via Env, and no zone stitching runs again.
    esc = geo.get("escape_pads")
    if esc:
        multi = {}
        for d in esc:
            multi.setdefault(d["net"], []).append(Pad(d))
        multi = {n: ps for n, ps in multi.items() if len(ps) >= 2}
        zone_map = {}
    else:
        multi = {n: ps for n, ps in nets.items() if len(ps) >= 2}
        zone_map = geo.get("zone_map") or {}
    repair_multi = {n: ps for n, ps in nets.items() if len(ps) >= 2}
    protected_nets = set(zone_map)
    protected_nets.update(geo.get("protected_zone_map") or {})

    base_order = sorted((n for n in multi if n not in zone_map),
                        key=lambda n: _span(multi, n))
    if not base_order:
        items, failures, stats = _attempt(geo, via_cost, multi, zone_map,
                                          [], verbose, 0,
                                          repair_multi=repair_multi,
                                          protected_nets=protected_nets)
        stats["partial"] = bool(failures)
        return {"items": items, "failures": failures, "stats": stats}

    best = None
    t_start = time.monotonic()
    board_deadline = t_start + BOARD_TIME_BUDGET
    if deadline is not None:
        board_deadline = min(board_deadline, deadline)
    available_budget = max(0.0, board_deadline - t_start)
    # ``deadline`` is a stage cap supplied by route_board_file (the main
    # geometry pass gets only part of BOARD_TIME_BUDGET so zone recovery has
    # time afterward). Derive all sub-budgets from that actual cap. Previously
    # primary_deadline could equal a short stage deadline, consuming the whole
    # stage before blocker recovery or alternate ordering ran.
    if esc:
        ripup_budget = min(RIPUP_TOTAL_BUDGET, available_budget * 0.20)
        alternate_budget = 0.0
    else:
        ripup_budget = min(RIPUP_TOTAL_BUDGET, available_budget * 0.25)
        alternate_budget = min(ALTERNATE_ORDER_BUDGET,
                               available_budget * 0.25)
    primary_budget = min(
        PRIMARY_ROUTE_BUDGET,
        max(0.0, available_budget - ripup_budget - alternate_budget))
    primary_deadline = min(board_deadline, t_start + primary_budget)
    seen_orders = {}
    executed_attempts = 0

    def run(order, attempt_no, note="", ripup_budget=RIPUP_BUDGET,
            allow_ripup=True, route_deadline=None):
        nonlocal best, executed_attempts
        if time.monotonic() >= board_deadline:
            return best[1] if best is not None else {
                n: ["board_time_budget_exhausted"] for n in order}
        order_key = (tuple(order), float(ripup_budget), bool(allow_ripup))
        if order_key in seen_orders:
            if verbose:
                print(f"  [LE] --- skip retry {attempt_no}: {note or 'order'} "
                      "duplicates an executed net order ---")
            return seen_orders[order_key]
        if verbose and attempt_no:
            print(f"  [LE] --- retry {attempt_no}: {note} ---")
        attempt_started = time.perf_counter()
        items, failures, stats = _attempt(
            geo, via_cost, multi, zone_map, order, verbose, attempt_no,
            repair_multi=repair_multi, protected_nets=protected_nets,
            ripup_budget=ripup_budget, deadline=board_deadline,
            route_deadline=(board_deadline if route_deadline is None
                            else route_deadline),
            allow_ripup=allow_ripup)
        stats["attempt_seconds"] = round(
            time.perf_counter() - attempt_started, 6)
        executed_attempts += 1
        seen_orders[order_key] = failures
        # Connectivity dominates every quality metric. Via count and length
        # only distinguish attempts with exactly the same open connections.
        score = (sum(len(v) for v in failures.values()), len(failures),
                 stats["vias"], stats["wirelength"])
        cand = (score, failures, items, stats)
        if best is None or score < best[0]:
            best = cand
        if progress_json and best is not None:
            ck = {"items": best[2], "failures": best[1],
                  "stats": {**best[3], "attempts": executed_attempts,
                            "partial": True}}
            tmp = progress_json + ".ck"
            with open(tmp, "w") as f:
                json.dump(ck, f)
            os.replace(tmp, progress_json)
        return failures

    # One full pass includes targeted recovery, limited to two blocker
    # combinations per failed net and a shared short recovery-time budget.
    failures = run(base_order, 0,
                   ripup_budget=ripup_budget,
                   route_deadline=primary_deadline)
    alternate_stage_deadline = min(
        board_deadline,
        primary_deadline + ripup_budget + alternate_budget)
    alternate_count = max(1, min(MAX_ALTERNATE_ORDERS, 2))
    alternate_slice = alternate_budget / alternate_count
    if failures and len(base_order) > 1 and MAX_ALTERNATE_ORDERS >= 1:
        failed = [n for n in base_order if n in failures]
        rest = [n for n in base_order if n not in failures]
        alternate_deadline = min(
            alternate_stage_deadline,
            time.monotonic() + alternate_slice)
        failures = run(failed + rest, 1, "failed nets first",
                       ripup_budget=0.0, allow_ripup=False,
                       route_deadline=alternate_deadline)
    if failures and len(base_order) > 1 and MAX_ALTERNATE_ORDERS >= 2:
        alternate_deadline = min(
            alternate_stage_deadline,
            time.monotonic() + alternate_slice)
        span_order = sorted(base_order, key=lambda n: _span(multi, n),
                            reverse=True)
        failures = run(span_order, 2,
                       "longest-span first", ripup_budget=0.0,
                       allow_ripup=False,
                       route_deadline=alternate_deadline)

    _score, failures, items, stats = best
    stats["attempts"] = executed_attempts
    stats["completion_first"] = True
    stats["alternate_orders_tried"] = max(0, executed_attempts - 1)
    stats["max_ripup_combos_per_net"] = MAX_RIPUP_COMBOS_PER_NET
    stats["parallel_path_workers"] = MAX_PARALLEL_PATHS
    stats["primary_route_budget_seconds"] = round(primary_budget, 3)
    stats["ripup_total_budget_seconds"] = round(ripup_budget, 3)
    stats["alternate_order_budget_seconds"] = round(alternate_budget, 3)
    stats["board_time_budget_seconds"] = round(available_budget, 3)
    stats["net_time_budget_seconds"] = NET_TIME_BUDGET
    stats["termination_reason"] = (
        "complete" if not failures else
        "board_time_budget_exhausted" if time.monotonic() >= board_deadline else
        "retry_budget_or_candidates_exhausted")
    stats["routing_seconds"] = round(time.monotonic() - t_start, 6)
    stats["partial"] = bool(failures)
    return {"items": items, "failures": failures, "stats": stats}
