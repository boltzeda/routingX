"""Waypoint search, parallel anchor search, and multi-pad net chaining."""

import heapq
import math
import multiprocessing as mp
import time
from multiprocessing.connection import wait as wait_connections

from shapely.geometry import Point, LineString, box

from router_constants import *  # noqa: F403


def _route_pair(env, net, src, dst, window_r, max_vias=None,
                max_expansions=None, search_kind="full", deadline=None):
    """A* over waypoints. Returns [(layer, a, b), ...] where a via move is
    (None, p, p), or None if no clear path exists inside the window.

    ``max_vias`` is used only by the safe adaptive probes.  When it is None,
    the state key and costs are identical to the original full-board search.
    """
    obst = env.obst
    copper = env.copper
    goal_layers = set(dst.accessible(copper))
    goal = (dst.x, dst.y)
    expansion_limit = (MAX_EXPANSIONS if max_expansions is None
                       else int(max_expansions))
    expanded = 0

    def finish(path):
        stats = getattr(env, "search_stats", None)
        if stats is not None:
            stats["search_calls"] += 1
            stats["search_expansions"] += expanded
            if search_kind == "adaptive":
                stats["adaptive_searches"] += 1
                stats["adaptive_expansions"] += expanded
        return path

    if deadline is not None and time.monotonic() >= deadline:
        return finish(None)

    base_d = math.hypot(dst.x - src.x, dst.y - src.y)
    window = box(min(src.x, dst.x) - window_r, min(src.y, dst.y) - window_r,
                 max(src.x, dst.x) + window_r, max(src.y, dst.y) + window_r)
    window = window.intersection(env.board_box)
    wx1, wy1, wx2, wy2 = window.bounds
    kx1, ky1, kx2, ky2 = env.keep_wire.bounds

    def endpoint_wire_clear(layer, a, b):
        """Check only copper newly introduced outside endpoint pads.

        A track begins at a pad centre, so its initial copper is already
        contained by that same-net pad. Testing the complete centreline
        against inflated neighbouring pads can falsely reject a legal
        outward escape at fine pitch (especially a wide power trace). When
        the requested track fits within the endpoint pad's minimum dimension,
        trim the centreline portion inside that pad and collision-test every
        remaining line component exactly. The emitted track still starts at
        the pad centre; this changes obstacle accounting, not geometry.
        """
        geom = LineString([a, b])
        for pad, point, other in ((src, a, b), (dst, b, a)):
            if abs(point[0] - pad.x) > 1e-7 \
                    or abs(point[1] - pad.y) > 1e-7:
                continue
            if env.trace_w > min(pad.w, pad.h) + EPS:
                continue
            base = pad.rmax + obst.margin + 0.05
            dx, dy = other[0] - pad.x, other[1] - pad.y
            outward = (
                (pad.x + base > kx2 and dx > 0.0 and abs(dx) >= abs(dy))
                or (pad.x - base < kx1 and dx < 0.0 and abs(dx) >= abs(dy))
                or (pad.y + base > ky2 and dy > 0.0 and abs(dy) >= abs(dx))
                or (pad.y - base < ky1 and dy < 0.0 and abs(dy) >= abs(dx))
            )
            if not outward:
                continue
            geom = geom.difference(pad.rect())
            if geom.is_empty:
                return True

        pending = [geom]
        while pending:
            part = pending.pop()
            if part.is_empty:
                continue
            if part.geom_type == "LineString":
                coords = list(part.coords)
                if len(coords) >= 2 and part.length > EPS \
                        and not obst.wire_clear(
                            layer, tuple(coords[0]), tuple(coords[-1])):
                    return False
            elif hasattr(part, "geoms"):
                pending.extend(part.geoms)
        return True

    # line-search first move: a clear direct segment needs no search at all
    # (the overwhelming majority of pairs on real boards).
    if kx1 <= min(src.x, goal[0]) and max(src.x, goal[0]) <= kx2 \
            and ky1 <= min(src.y, goal[1]) and max(src.y, goal[1]) <= ky2:
        for layer in set(src.accessible(copper)) & goal_layers:
            if (endpoint_wire_clear(layer, (src.x, src.y), goal)
                    and env.zone_crossing_cost(
                        net, layer, (src.x, src.y), goal) <= EPS):
                return finish([(layer, (src.x, src.y), goal)])

    # stage 1 (per pair): static waypoint set per layer inside the window.
    # extra (escape points + goal) is ALWAYS a candidate — the nearest-
    # MAX_CORNERS cut applies to corner points only. Corners go into a
    # uniform grid so the nearest-k lookup per expansion does not sort the
    # whole set (that is what made dense boards crawl).
    extra = env.escape_points(src) + env.escape_points(dst) + [goal]
    corners = {}
    grids = {}
    for layer in copper:
        pts = [c for c in obst.layer_points(layer)
               if wx1 <= c[0] <= wx2 and wy1 <= c[1] <= wy2]
        corners[layer] = pts
        g = {}
        ix0 = iy0 = 10 ** 9
        ix1 = iy1 = -10 ** 9
        for c in pts:
            gx, gy = int(c[0] // GRID_CELL), int(c[1] // GRID_CELL)
            g.setdefault((gx, gy), []).append(c)
            ix0, ix1 = min(ix0, gx), max(ix1, gx)
            iy0, iy1 = min(iy0, gy), max(iy1, gy)
        grids[layer] = (g, (ix0, ix1, iy0, iy1))

    def nearest_corners(layer, p, k):
        pts = corners[layer]
        if len(pts) <= k:
            return pts
        g, (ix0, ix1, iy0, iy1) = grids[layer]
        ix, iy = int(p[0] // GRID_CELL), int(p[1] // GRID_CELL)
        rmax = max(ix1 - ix, ix - ix0, iy1 - iy, iy - iy0)
        collected = []
        r = 0
        while r <= rmax:
            for gx in range(max(ix0, ix - r), min(ix1, ix + r) + 1):
                for gy in range(max(iy0, iy - r), min(iy1, iy + r) + 1):
                    if r and abs(gx - ix) != r and abs(gy - iy) != r:
                        continue          # ring cells only
                    cell = g.get((gx, gy))
                    if cell:
                        collected.extend(cell)
            if len(collected) >= k:
                break
            r += 1
        collected.sort(key=lambda c: (c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2)
        return collected[:k]

    wire_memo = {}
    via_memo = {}
    zone_cost_memo = {}
    # keep-in boxes are convex: a segment is inside iff BOTH endpoints are
    # inside — a pure-float bounds check, no shapely objects (this test ran
    # ~1M times per hard pair and dominated the profile). kx*/ky* come from
    # the direct-segment pre-check above.
    vx1, vy1, vx2, vy2 = env.keep_via.bounds

    def keep_seg_inside(a, b):
        return (min(a[0], b[0]) >= kx1 and max(a[0], b[0]) <= kx2
                and min(a[1], b[1]) >= ky1 and max(a[1], b[1]) <= ky2)

    def via_ok(p):
        k = (round(p[0], 3), round(p[1], 3))
        v = via_memo.get(k)
        if v is None:
            v = (vx1 <= p[0] <= vx2 and vy1 <= p[1] <= vy2
                 and env.hole_ok(p)
                 and obst.via_clear(copper, p[0], p[1], env.via_r))
            via_memo[k] = v
        return v

    def wire_clear(layer, a, b):
        k = (layer, round(a[0], 3), round(a[1], 3), round(b[0], 3), round(b[1], 3))
        v = wire_memo.get(k)
        if v is None:
            v = endpoint_wire_clear(layer, a, b)
            wire_memo[k] = v
        return v

    def zone_crossing_cost(layer, a, b):
        k = (layer, round(a[0], 3), round(a[1], 3),
             round(b[0], 3), round(b[1], 3))
        value = zone_cost_memo.get(k)
        if value is None:
            value = env.zone_crossing_cost(net, layer, a, b)
            zone_cost_memo[k] = value
        return value

    def h(layer, p):
        base = math.hypot(goal[0] - p[0], goal[1] - p[1])
        return base + (env.via_cost if layer not in goal_layers else 0.0)

    def state_key(layer, p, vias_used):
        base = (layer, round(p[0], 3), round(p[1], 3))
        # A constrained search must retain a longer, lower-via arrival at the
        # same point; it may be the only state still able to reach the goal
        # without exceeding the budget. The unrestricted fallback deliberately
        # keeps the original three-field key and therefore its original search.
        return base + (vias_used,) if max_vias is not None else base

    counter = 0
    open_heap, gscore, came = [], {}, {}
    for layer in src.accessible(copper):
        k = state_key(layer, (src.x, src.y), 0)
        if k in gscore:
            continue
        gscore[k] = 0.0
        heapq.heappush(open_heap, (h(layer, (src.x, src.y)), counter,
                                   layer, (src.x, src.y), 0))
        counter += 1

    while (open_heap and expanded < expansion_limit
           and (deadline is None or time.monotonic() < deadline)):
        _f, _c, layer, p, vias_used = heapq.heappop(open_heap)
        k = state_key(layer, p, vias_used)
        g_here = gscore.get(k, math.inf)
        if g_here == math.inf:
            continue
        if round(p[0], 3) == round(goal[0], 3) and round(p[1], 3) == round(goal[1], 3) \
                and layer in goal_layers:
            path = []
            cur = k
            while cur in came:
                prev, kind = came[cur]
                if kind == "seg":
                    path.append((prev[0], (prev[1], prev[2]), (cur[1], cur[2])))
                else:
                    path.append((None, (cur[1], cur[2]), (cur[1], cur[2])))
                cur = prev
            path.reverse()
            return finish(path)
        expanded += 1

        neighbors = []  # (cost, nlayer, npoint, kind)

        if (max_vias is None or vias_used < max_vias) and via_ok(p):
            for l2 in copper:
                if l2 != layer:
                    neighbors.append((env.via_cost, l2, p, "via"))

        if layer in goal_layers:
            neighbors.append((math.hypot(goal[0] - p[0], goal[1] - p[1]),
                              layer, goal, "seg"))

        cands = corners[layer]
        if len(cands) > MAX_CORNERS:
            cands = nearest_corners(layer, p, MAX_CORNERS)
        seen = set()
        for c in list(cands) + extra:
            ck = (round(c[0], 3), round(c[1], 3))
            if ck in seen:
                continue
            seen.add(ck)
            if abs(c[0] - p[0]) < 1e-6 and abs(c[1] - p[1]) < 1e-6:
                continue
            neighbors.append((math.hypot(c[0] - p[0], c[1] - p[1]), layer, c, "seg"))

        for cost, nlayer, np, kind in neighbors:
            if kind == "seg":
                if not keep_seg_inside(p, np):
                    continue
                if not wire_clear(layer, p, np):
                    continue
            next_vias = vias_used + (1 if kind == "via" else 0)
            nk = state_key(nlayer, np, next_vias)
            ng = g_here + cost
            if kind == "seg":
                ng += zone_crossing_cost(layer, p, np)
            if ng < gscore.get(nk, math.inf) - 1e-9:
                gscore[nk] = ng
                came[nk] = (k, kind)
                counter += 1
                heapq.heappush(open_heap, (ng + h(nlayer, np), counter,
                                           nlayer, np, next_vias))
    return finish(None)


def _route_pair_adaptive(env, net, src, dst, deadline=None):
    """Paper-inspired local regions with a no-extra-via acceptance guard.

    A local result is accepted only with the theoretical minimum via count:
    zero when the pads share an accessible layer, otherwise one. This prevents
    the restricted region from preferring a vertical transition over a wider
    planar detour (the via increase reported by the paper's scope study).
    Failure of the bounded probes always reaches the original 1e6-radius,
    unrestricted, 12000-expansion search unchanged.
    """
    # On small layouts the full waypoint set is cheap, while constructing a
    # second local set is measurable overhead. This gate changes no search
    # result; it selects the original implementation immediately.
    if env.escape_mode or len(env.pads) < ADAPTIVE_MIN_PADS:
        env.search_stats["adaptive_bypassed"] += 1
        return _route_pair(env, net, src, dst, 1e6, deadline=deadline)

    copper = env.copper
    shared_layers = set(src.accessible(copper)) & set(dst.accessible(copper))
    minimum_vias = 0 if shared_layers else 1
    bx1, by1, bx2, by2 = env.board_box.bounds
    sx1, sx2 = sorted((src.x, dst.x))
    sy1, sy2 = sorted((src.y, dst.y))
    full_margin = max(sx1 - bx1, bx2 - sx2, sy1 - by1, by2 - sy2)
    span = math.hypot(dst.x - src.x, dst.y - src.y)
    rule_margin = 4.0 * (env.clearance + env.trace_w + env.via_size)
    radius = max(2.0, rule_margin, min(8.0, 0.2 * span))

    tried = set()
    for _ in range(ADAPTIVE_WINDOWS):
        radius = round(radius, 6)
        if radius >= full_margin - EPS or radius in tried:
            break
        tried.add(radius)
        path = _route_pair(
            env, net, src, dst, radius,
            max_vias=minimum_vias,
            max_expansions=ADAPTIVE_EXPANSIONS,
            search_kind="adaptive", deadline=deadline)
        if path is not None:
            env.search_stats["adaptive_hits"] += 1
            return path
        radius *= 2.0

    env.search_stats["fullboard_fallbacks"] += 1
    return _route_pair(env, net, src, dst, 1e6, deadline=deadline)


def _emit_path(env, net, path):
    """Emit tracks/vias for an A* path, merging collinear same-layer runs."""
    merged = []
    for layer, a, b in path:
        if layer is None:
            merged.append((None, a, b))
            continue
        if merged and merged[-1][0] == layer:
            pa, pb = merged[-1][1], merged[-1][2]
            d1 = (pb[0] - pa[0], pb[1] - pa[1])
            d2 = (b[0] - pb[0], b[1] - pb[1])
            n1, n2 = math.hypot(*d1), math.hypot(*d2)
            if n1 > 0 and n2 > 0 and \
                    abs((d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2) - 1.0) < 1e-6:
                merged[-1] = (layer, pa, b)
                continue
        merged.append((layer, a, b))
    for layer, a, b in merged:
        if layer is None:
            env.emit_via(net, a)
        elif math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
            env.emit_track(net, layer, a, b)


def _dist_to_segs(p, segs):
    best = math.inf
    for a, b in segs:
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        if L2 == 0:
            d = math.hypot(p[0] - a[0], p[1] - a[1])
        else:
            t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
            d = math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))
        best = min(best, d)
    return best if segs else 0.0


def _path_segments(path):
    segs = []
    for layer, a, b in path:
        if layer is not None and math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
            segs.append((a, b))
    return segs


def _route_pair_process(conn, env, net, src, dst, deadline):
    """Fork worker for one independent anchor-to-pad route search."""
    try:
        path = _route_pair_adaptive(env, net, src, dst, deadline=deadline)
        conn.send((path, env.search_stats, env.obst.blocker_hits, None))
    except BaseException as exc:
        try:
            conn.send((None, env.search_stats, env.obst.blocker_hits,
                       f"{type(exc).__name__}: {exc}"))
        except Exception:
            pass
    finally:
        conn.close()


def _route_pair_parallel(env, net, candidates, dst, deadline):
    """Try independent anchors in isolated processes and take the first path.

    Each worker inherits an immutable snapshot of the current route state.
    Only the winning path is committed by the parent, so concurrent searches
    cannot cross or mutate one another's copper.
    """
    if len(candidates) <= 1 or "fork" not in mp.get_all_start_methods():
        for src in candidates:
            path = _route_pair_adaptive(env, net, src, dst, deadline=deadline)
            if path is not None:
                return path
        return None

    ctx = mp.get_context("fork")
    baseline = dict(env.search_stats)
    pending = {}
    processes = []
    winner = None
    worker_errors = 0
    aggregate = {k: 0 for k in baseline}
    aggregate_blockers = {}
    candidates = candidates[:MAX_PARALLEL_PATHS]
    env.search_stats["parallel_path_candidates"] += len(candidates)
    env.search_stats["parallel_path_workers_peak"] = max(
        env.search_stats["parallel_path_workers_peak"], len(candidates))
    try:
        for src in candidates:
            if time.monotonic() >= deadline:
                break
            recv_conn, send_conn = ctx.Pipe(duplex=False)
            proc = ctx.Process(
                target=_route_pair_process,
                args=(send_conn, env, net, src, dst, deadline))
            try:
                proc.start()
            except Exception:
                recv_conn.close()
                send_conn.close()
                worker_errors += 1
                continue
            send_conn.close()
            pending[recv_conn] = proc
            processes.append(proc)

        while pending and time.monotonic() < deadline:
            ready = wait_connections(
                list(pending), timeout=max(0.0, deadline - time.monotonic()))
            if not ready:
                break
            for conn in ready:
                proc = pending.pop(conn)
                try:
                    path, worker_stats, worker_blockers, error = conn.recv()
                    if error:
                        worker_errors += 1
                    for key in aggregate:
                        aggregate[key] += max(
                            0, worker_stats.get(key, baseline[key])
                            - baseline[key])
                    if path is None:
                        for blocker, hits in worker_blockers.items():
                            aggregate_blockers[blocker] = (
                                aggregate_blockers.get(blocker, 0) + hits)
                    if path is not None:
                        winner = path
                except (EOFError, OSError):
                    pass
                finally:
                    conn.close()
                proc.join(timeout=0.01)
                if winner is not None:
                    break
            if winner is not None:
                break
            for conn, proc in list(pending.items()):
                if not proc.is_alive():
                    pending.pop(conn)
                    conn.close()
                    proc.join(timeout=0.01)
    finally:
        for conn, proc in list(pending.items()):
            conn.close()
            if proc.is_alive():
                proc.terminate()
            proc.join(timeout=0.2)
            if proc.is_alive() and hasattr(proc, "kill"):
                proc.kill()
                proc.join(timeout=0.2)
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
            proc.join(timeout=0.2)
            if proc.is_alive() and hasattr(proc, "kill"):
                proc.kill()
                proc.join(timeout=0.2)

    for key, delta in aggregate.items():
        env.search_stats[key] = baseline[key] + delta
    if winner is None:
        for blocker, hits in aggregate_blockers.items():
            env.obst.blocker_hits[blocker] = (
                env.obst.blocker_hits.get(blocker, 0) + hits)
    if winner is not None:
        env.search_stats["parallel_path_wins"] += 1
    env.search_stats["parallel_path_worker_errors"] += worker_errors
    if winner is None and worker_errors >= len(processes) \
            and time.monotonic() < deadline:
        for src in candidates:
            path = _route_pair_adaptive(env, net, src, dst,
                                        deadline=deadline)
            if path is not None:
                return path
    return winner


def _route_chained_net(env, net, pads, deadline=None):
    """Pairwise chaining: nearest-unconnected pad joins the routed tree.
    Returns (failures, pairs_routed)."""
    connected = [pads[0]]
    tree_segs = []
    remaining = list(pads[1:])
    failures, pairs = [], 0
    net_deadline = time.monotonic() + NET_TIME_BUDGET
    if deadline is not None:
        net_deadline = min(net_deadline, deadline)
    while remaining:
        if time.monotonic() >= net_deadline:
            failures.extend(
                f"{p.ref}@{p.x:.1f},{p.y:.1f} (net_time_budget_exhausted)"
                for p in remaining)
            break
        u = min(remaining, key=lambda r: _dist_to_segs((r.x, r.y), tree_segs))
        anchors = sorted(
            connected, key=lambda c: math.hypot(c.x - u.x, c.y - u.y))
        if len(anchors) > 1 and time.monotonic() < net_deadline:
            quick_deadline = min(net_deadline, time.monotonic() + 0.05)
            path = _route_pair_adaptive(
                env, net, anchors[0], u, deadline=quick_deadline)
            if path is not None:
                _emit_path(env, net, path)
                tree_segs.extend(_path_segments(path))
                pairs += 1
                connected.append(u)
                remaining.remove(u)
                continue
        path = _route_pair_parallel(
            env, net, anchors[:MAX_PARALLEL_PATHS], u, net_deadline)
        if path is None:
            why = (" (net_time_budget_exhausted)"
                   if time.monotonic() >= net_deadline else "")
            failures.append(f"{u.ref}@{u.x:.1f},{u.y:.1f}{why}")
        else:
            _emit_path(env, net, path)
            tree_segs.extend(_path_segments(path))
            pairs += 1
            connected.append(u)
        remaining.remove(u)
    return failures, pairs
