"""LineExplore routing orchestration and worker-stage command handling."""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import traceback

from routing_rules import normalize_routing_rules
from router_constants import *  # noqa: F403
from kicad_host import subprocess_environment
from board_geometry import _clamp_routing_rules_for_board, extract_geometry
from route_planner import route_geometry
from route_refinement import postprocess_135_routes
from board_apply import apply_routes
from board_audit import (
    _audit_subprocess, audit_zone_pads, _native_kicad_drc,
    _new_drc_findings, _audit_final_connectivity,
)

WORKER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "worker.py")


def _geometry_stage(geo_json, routes_json, via_cost, deadline):
    import faulthandler
    faulthandler.enable()     # post-mortem traceback if the C layer crashes
    with open(geo_json) as f:
        geo = json.load(f)
    result = route_geometry(geo, via_cost=via_cost,
                            progress_json=routes_json,
                            deadline=float(deadline))
    with open(routes_json, "w") as f:
        json.dump(result, f)
    return 0 if not result["failures"] else 2


def _run_geometry_subprocess(geo, routes_json, via_cost, deadline=None):
    """Phase B in a clean interpreter so shapely never coexists with pcbnew."""
    here = os.path.dirname(WORKER_SCRIPT)
    routes_json = os.path.abspath(routes_json)
    geo_json = routes_json + ".geo"
    if deadline is None:
        deadline = time.monotonic() + BOARD_TIME_BUDGET
    if time.monotonic() >= deadline:
        pads = geo.get("escape_pads") or geo.get("pads", [])
        by_net = {}
        for pad in pads:
            by_net.setdefault(pad["net"], []).append(pad)
        result = {
            "items": list(geo.get("pre_items") or []),
            "failures": {
                net: [f"{p.get('ref', '?')}@{p['x']:.1f},{p['y']:.1f} "
                      "(board_time_budget_exhausted)" for p in net_pads]
                for net, net_pads in by_net.items() if len(net_pads) >= 2
            },
            "stats": {
                "attempts": 0, "partial": True,
                "termination_reason": "board_time_budget_exhausted",
                "routing_seconds": 0.0,
                "board_time_budget_seconds": BOARD_TIME_BUDGET,
            },
        }
        with open(routes_json, "w") as f:
            json.dump(result, f)
        return result
    with open(geo_json, "w") as f:
        json.dump(geo, f)
    try:
        os.remove(routes_json)      # stale checkpoint from an earlier run
    except OSError:
        pass
    proc = subprocess.run(
        [sys.executable, "-u", WORKER_SCRIPT,
         "--geometry-stage", geo_json, routes_json, str(via_cost),
         str(deadline)],
        env=subprocess_environment(), cwd=here)   # inherit stdio: [LE] progress streams live
    if proc.returncode not in (0, 2):
        # crash mid-retry: the per-attempt checkpoint is still a usable board
        if os.path.exists(routes_json):
            try:
                with open(routes_json) as f:
                    result = json.load(f)
                if result.get("items"):
                    print(f"  [LE] WARNING: geometry stage exited "
                          f"{proc.returncode}; using checkpoint "
                          f"({len(result.get('failures', {}))} nets unrouted "
                          f"after {result.get('stats', {}).get('attempts')} "
                          f"attempts)")
                    return result
            except Exception:
                pass
        raise RuntimeError(f"geometry stage exited {proc.returncode}")
    with open(routes_json) as f:
        return json.load(f)


def _postprocess_stage(geo_json, routes_json, via_cost):
    """Normalize the final combined route set in a Shapely-only process."""
    import faulthandler
    faulthandler.enable()
    with open(geo_json) as f:
        geo = json.load(f)
    with open(routes_json) as f:
        result = json.load(f)
    result = postprocess_135_routes(geo, result, via_cost=via_cost)
    tmp_json = routes_json + ".post.tmp"
    with open(tmp_json, "w") as f:
        json.dump(result, f)
    os.replace(tmp_json, routes_json)
    return 0


def _run_postprocess_subprocess(geo, routes_json, via_cost):
    """Run 135-degree conversion once after all raw escape passes finish.

    The raw route JSON remains a usable checkpoint if the process itself
    fails. Segment-level geometry errors are contained by
    postprocess_135_routes and therefore normally return a successful result
    with failure statistics.
    """
    here = os.path.dirname(WORKER_SCRIPT)
    routes_json = os.path.abspath(routes_json)
    geo_json = routes_json + ".post.geo"
    with open(geo_json, "w") as f:
        json.dump(geo, f)
    try:
        proc = subprocess.run(
            [sys.executable, "-u", WORKER_SCRIPT,
             "--postprocess-stage", geo_json, routes_json, str(via_cost)],
            env=subprocess_environment(), cwd=here)
        if proc.returncode != 0:
            with open(routes_json) as f:
                result = json.load(f)
            stats = result.setdefault("stats", {})
            stats.update({"postprocess_135": False,
                          "postprocess_converted": 0,
                          "postprocess_added_segments": 0,
                          "postprocess_failures": 1})
            result["postprocess_failures"] = [
                f"post-processing stage exited {proc.returncode}; "
                "original route geometry retained"]
            tmp_json = routes_json + ".post.tmp"
            with open(tmp_json, "w") as f:
                json.dump(result, f)
            os.replace(tmp_json, routes_json)
            print(f"  [LE] WARNING: 135-degree post-process exited "
                  f"{proc.returncode}; original routes retained")
            return result
        with open(routes_json) as f:
            return json.load(f)
    finally:
        try:
            os.remove(geo_json)
        except OSError:
            pass


def _load_drc(output_dir, prefix, drc=None):
    if drc:
        return normalize_routing_rules({**DEFAULT_DRC, **drc})
    path = os.path.join(output_dir, f"{prefix}_drc.json")
    if os.path.exists(path):
        try:
            with open(path) as f:
                return normalize_routing_rules({**DEFAULT_DRC, **json.load(f)})
        except Exception:
            pass
    return normalize_routing_rules(DEFAULT_DRC)


def _apply_subprocess(board_file, out_file, routes_json, drc):
    """Phase C in a fresh interpreter: apply_routes calls board.Remove(),
    which corrupts swig wrapping for the rest of its process — a second
    apply or audit in the same interpreter then crashes on raw SwigPyObject.
    Every apply gets its own process."""
    here = os.path.dirname(WORKER_SCRIPT)
    board_file = os.path.abspath(board_file)
    out_file = os.path.abspath(out_file)
    routes_json = os.path.abspath(routes_json)
    drc_json = out_file + ".apply.drc.json"
    with open(drc_json, "w") as f:
        json.dump(drc, f)
    proc = subprocess.run(
        [sys.executable, "-u", WORKER_SCRIPT,
         "--apply-stage", board_file, out_file, routes_json, drc_json],
        env=subprocess_environment(), cwd=here)
    if proc.returncode != 0:
        raise RuntimeError(f"apply stage exited {proc.returncode}")


def route_board_file(board_file, out_file, drc, via_cost=2.0, verbose=True):
    """Full A->B->C cycle on a standalone board file. Returns (ok, stats)."""
    route_started = time.monotonic()
    drc = _clamp_routing_rules_for_board(board_file, drc)
    search_deadline = route_started + BOARD_TIME_BUDGET
    geo = extract_geometry(board_file, drc, verbose=verbose)
    routes_json = out_file + ".routes.json"
    # Leave time for the post-refill zone audit to identify pads the pours did
    # not actually connect, then route those as ordinary same-net escapes.
    # Previously the general search could consume the entire board budget,
    # making this connectivity-repair loop a no-op on dense boards.
    geometry_deadline = search_deadline
    if geo.get("zone_map"):
        geometry_deadline = min(
            search_deadline,
            route_started + max(0.0, BOARD_TIME_BUDGET - ZONE_ESCAPE_BUDGET))
    routes = _run_geometry_subprocess(
        geo, routes_json, via_cost, deadline=geometry_deadline)
    stats = routes.get("stats", {})
    stats["zone_escape_budget_seconds"] = (
        ZONE_ESCAPE_BUDGET if geo.get("zone_map") else 0.0)
    failures = {n: list(v) for n, v in
                (routes.get("failures") or {}).items()}
    _apply_subprocess(board_file, out_file, routes_json,
                      geo.get("drc") or drc)  # board-clamped
    print(f"  [LE] applied routes -> {out_file}")

    # Zone-escape loop: the refill decides real connectivity. Any zone-net
    # pad it could not reach gets a second routing pass as an ordinary
    # trace (with everything emitted so far as obstacles), then the zones
    # refill again. Every round shares the same run deadline.
    cur_items = routes.get("items", [])
    escape_reports = []
    try:
        current_drc_report = _native_kicad_drc(out_file)
    except Exception as exc:
        current_drc_report = None
        stats["zone_escape_drc_audit_error"] = (
            f"{type(exc).__name__}: {exc}")
    for esc_round in range(1, 5):
        if time.monotonic() >= search_deadline:
            stats["zone_escape_budget_exhausted"] = True
            break
        if current_drc_report is None:
            stats["zone_escape_stopped"] = "native_drc_audit_unavailable"
            break
        try:
            esc_pads = _audit_subprocess(out_file, geo)
        except Exception as exc:
            print(f"  [LE] zone audit failed ({exc}) — skipping escapes.")
            break
        if not esc_pads:
            break
        print(f"  [LE] zone escape round {esc_round}: {len(esc_pads)} pad(s) "
              f"unreached by the fill -> routing escape traces")
        esc_geo = {**geo, "escape_pads": esc_pads,
                   "zone_map": {}, "pre_items": cur_items,
                   "protected_zone_map": geo.get("zone_map") or {}}
        esc_json = out_file + f".esc{esc_round}.routes.json"
        esc = _run_geometry_subprocess(
            esc_geo, esc_json, via_cost, deadline=search_deadline)
        esc_stats = esc.get("stats", {})
        escape_reports.append({
            "round": esc_round,
            "failed_nets": sorted(esc.get("failures") or {}),
            "failed_connections": esc.get("failures") or {},
            "blocked_routes": esc_stats.get("blocked_routes", []),
            "ripup_fixed": esc_stats.get("ripup_fixed", 0),
            "ripup_improved_commits": esc_stats.get(
                "ripup_improved_commits", 0),
            "ripup_combo_attempts": esc_stats.get("ripup_combo_attempts", 0),
            "ripup_rollbacks": esc_stats.get("ripup_rollbacks", 0),
            "ripup_transaction_errors": esc_stats.get(
                "ripup_transaction_errors", 0),
            "blocker_recovery_source": esc_stats.get(
                "blocker_recovery_source"),
            "blocker_hint_net_count": esc_stats.get(
                "blocker_hint_net_count", 0),
            "blocker_hint_observations": esc_stats.get(
                "blocker_hint_observations", 0),
        })
        new_items = [it for it in esc.get("items", []) if it not in cur_items]
        if not new_items:
            print("  [LE] escape pass produced nothing new — stopping.")
            break
        before_zone_components = sum(
            pad.get("escape_role") == "unconnected" for pad in esc_pads)
        before_open_items = len(current_drc_report.get("unconnected_items") or [])
        candidate_dir = None
        try:
            with tempfile.TemporaryDirectory(
                    prefix=(f".{os.path.splitext(os.path.basename(out_file))[0]}"
                            "_zone_escape_"),
                    dir=os.path.dirname(os.path.abspath(out_file))) as candidate_dir:
                candidate_board = os.path.join(
                    candidate_dir, os.path.basename(out_file))
                project_name = os.path.basename(out_file).replace(
                    ".kicad_pcb", ".kicad_pro")
                project_source = os.path.join(
                    os.path.dirname(os.path.abspath(out_file)), project_name)
                if os.path.isfile(project_source):
                    shutil.copy2(project_source,
                                 os.path.join(candidate_dir, project_name))
                _apply_subprocess(out_file, candidate_board, esc_json,
                                  geo.get("drc") or drc)
                candidate_drc = _native_kicad_drc(candidate_board)
                candidate_zone_pads = _audit_subprocess(candidate_board, geo)
                after_zone_components = sum(
                    pad.get("escape_role") == "unconnected"
                    for pad in candidate_zone_pads)
                after_open_items = len(
                    candidate_drc.get("unconnected_items") or [])
                added_errors = _new_drc_findings(
                    current_drc_report, candidate_drc, severity="error")
                added_isolated = _new_drc_findings(
                    current_drc_report, candidate_drc,
                    severity="warning", kind="isolated_copper")
                connectivity_improved = (
                    after_open_items <= before_open_items
                    and after_zone_components < before_zone_components)
                accepted = (not added_errors and not added_isolated
                            and connectivity_improved)
                escape_reports.append({
                    "round": esc_round,
                    "failed_nets": sorted(esc.get("failures") or {}),
                    "failed_connections": esc.get("failures") or {},
                    "blocked_routes": esc_stats.get("blocked_routes", []),
                    "ripup_fixed": esc_stats.get("ripup_fixed", 0),
                    "ripup_improved_commits": esc_stats.get(
                        "ripup_improved_commits", 0),
                    "ripup_combo_attempts": esc_stats.get(
                        "ripup_combo_attempts", 0),
                    "ripup_rollbacks": esc_stats.get("ripup_rollbacks", 0),
                    "ripup_transaction_errors": esc_stats.get(
                        "ripup_transaction_errors", 0),
                    "blocker_recovery_source": esc_stats.get(
                        "blocker_recovery_source"),
                    "blocker_hint_net_count": esc_stats.get(
                        "blocker_hint_net_count", 0),
                    "blocker_hint_observations": esc_stats.get(
                        "blocker_hint_observations", 0),
                    "zone_components_before": before_zone_components,
                    "zone_components_after": after_zone_components,
                    "open_items_before": before_open_items,
                    "open_items_after": after_open_items,
                    "new_drc_errors": len(added_errors),
                    "new_isolated_copper_warnings": len(added_isolated),
                    "accepted": accepted,
                })
                if not accepted:
                    stats["zone_escape_stopped"] = (
                        "new_drc_error" if added_errors else
                        "new_isolated_copper" if added_isolated else
                        "connectivity_not_improved")
                    print("  [LE] escape candidate rejected: "
                          f"{stats['zone_escape_stopped']} "
                          f"({before_zone_components} -> "
                          f"{after_zone_components} zone components, "
                          f"{len(added_errors)} new errors, "
                          f"{len(added_isolated)} new isolated-copper warnings)")
                    break
                shutil.copy2(candidate_board, out_file)
                cur_items = esc.get("items", [])
                current_drc_report = candidate_drc
                print(f"  [LE] accepted zone escape ({len(new_items)} new item(s)); "
                      f"unconnected zone components "
                      f"{before_zone_components} -> {after_zone_components}")
        except Exception as exc:
            stats["zone_escape_candidate_error"] = (
                f"{type(exc).__name__}: {exc}")
            stats["zone_escape_stopped"] = "candidate_audit_failed"
            print(f"  [LE] zone escape candidate audit failed ({exc}) — "
                  "keeping the prior board.")
            break

    # Shape normalization is deliberately deferred until every zone-escape
    # pass is complete. Keep the route failures and attempt statistics,
    # combine them with the final raw items, then refine and apply once.
    stats["escape_passes"] = escape_reports
    stats["escape_blocked_routes"] = [
        diagnostic
        for report in escape_reports
        for diagnostic in report["blocked_routes"]
    ]
    final_result = {"items": cur_items, "failures": failures, "stats": stats}
    tmp_json = routes_json + ".raw.tmp"
    with open(tmp_json, "w") as f:
        json.dump(final_result, f)
    os.replace(tmp_json, routes_json)
    final_result = _run_postprocess_subprocess(geo, routes_json, via_cost)
    stats = final_result.get("stats", stats)
    # Validate shape normalization on a disposable board. Moving a route
    # around an obstacle can change the zone refill even when the raw escape
    # candidate passed DRC, so only commit the shaped result if native KiCad
    # confirms it preserves the raw board's connectivity and zone warnings.
    final_drc_report = None
    try:
        raw_drc = _native_kicad_drc(out_file)
        final_drc_report = raw_drc
        raw_zone_pads = _audit_subprocess(out_file, geo)
        raw_zone_components = sum(
            pad.get("escape_role") == "unconnected"
            for pad in raw_zone_pads)
        with tempfile.TemporaryDirectory(
                prefix=(f".{os.path.splitext(os.path.basename(out_file))[0]}"
                        "_shape_check_"),
                dir=os.path.dirname(os.path.abspath(out_file))) as shape_dir:
            shape_board = os.path.join(shape_dir, os.path.basename(out_file))
            project_name = os.path.basename(out_file).replace(
                ".kicad_pcb", ".kicad_pro")
            project_source = os.path.join(
                os.path.dirname(os.path.abspath(out_file)), project_name)
            if os.path.isfile(project_source):
                shutil.copy2(project_source,
                             os.path.join(shape_dir, project_name))
            _apply_subprocess(out_file, shape_board, routes_json,
                              geo.get("drc") or drc)
            shaped_drc = _native_kicad_drc(shape_board)
            shaped_zone_pads = _audit_subprocess(shape_board, geo)
            shaped_zone_components = sum(
                pad.get("escape_role") == "unconnected"
                for pad in shaped_zone_pads)
            added_shape_errors = _new_drc_findings(
                raw_drc, shaped_drc, severity="error")
            added_shape_isolated = _new_drc_findings(
                raw_drc, shaped_drc, severity="warning",
                kind="isolated_copper")
            raw_open_items = len(raw_drc.get("unconnected_items") or [])
            shaped_open_items = len(
                shaped_drc.get("unconnected_items") or [])
            shape_safe = (
                not added_shape_errors
                and not added_shape_isolated
                and shaped_open_items <= raw_open_items
                and shaped_zone_components <= raw_zone_components)
            stats["postprocess_drc_validation"] = {
                "accepted": shape_safe,
                "raw_open_items": raw_open_items,
                "shaped_open_items": shaped_open_items,
                "raw_zone_components": raw_zone_components,
                "shaped_zone_components": shaped_zone_components,
                "new_drc_errors": len(added_shape_errors),
                "new_isolated_copper_warnings": len(
                    added_shape_isolated),
            }
            if shape_safe:
                shutil.copy2(shape_board, out_file)
                final_drc_report = shaped_drc
                print(f"  [LE] applied DRC-safe 135-degree post-process -> {out_file}")
            else:
                attempted_converted = int(
                    stats.get("postprocess_converted", 0) or 0)
                attempted_added = int(
                    stats.get("postprocess_added_segments", 0) or 0)
                final_result["items"] = cur_items
                stats["postprocess_135_rejected"] = True
                stats["postprocess_rejected_converted"] = attempted_converted
                stats["postprocess_rejected_added_segments"] = attempted_added
                stats["postprocess_135"] = False
                stats["postprocess_converted"] = 0
                stats["postprocess_added_segments"] = 0
                stats["wirelength_post_135"] = stats.get(
                    "wirelength_pre_135")
                print("  [LE] rejected 135-degree post-process: it added "
                      f"{len(added_shape_errors)} native DRC error(s), "
                      f"{len(added_shape_isolated)} isolated-copper warning(s), "
                      "or worsened connectivity; keeping raw routes.")
    except Exception as exc:
        stats["postprocess_drc_validation_error"] = (
            f"{type(exc).__name__}: {exc}")
        stats["postprocess_135_rejected"] = True
        final_result["items"] = cur_items
        print(f"  [LE] post-process DRC audit failed ({exc}); "
              "keeping raw routes.")

    # The refill is the source of truth for plane connectivity. Replace
    # internal zone-net guesses with the final pad audit so a failed escape
    # attempt is reported as partial, while a successful refill clears it.
    zone_nets = set(geo.get("zone_map") or {})
    for net in zone_nets:
        failures.pop(net, None)
    try:
        remaining_zone_pads = _audit_subprocess(out_file, geo)
        bad_zone_pads = [p for p in remaining_zone_pads
                         if p.get("escape_role") == "unconnected"]
        for pad in bad_zone_pads:
            failures.setdefault(pad["net"], []).append(
                f"{pad['ref']}@{pad['x']:.1f},{pad['y']:.1f} (zone_unconnected)")
        stats["final_zone_unconnected_pads"] = len(bad_zone_pads)
    except Exception as exc:
        stats["final_zone_audit_error"] = f"{type(exc).__name__}: {exc}"

    try:
        connectivity = _audit_final_connectivity(
            out_file, board_file, report_override=final_drc_report)
        stats["final_unrouted_connections"] = connectivity[
            "unrouted_connections"]
        stats["final_unrouted_nets"] = connectivity["unrouted_nets"]
        stats["final_drc_violations"] = connectivity["drc_violations"]
        stats["baseline_drc_violations"] = connectivity[
            "baseline_drc_violations"]
        stats["new_drc_violations"] = connectivity["new_drc_violations"]
        stats["new_drc_violations_by_type"] = connectivity[
            "new_drc_violations_by_type"]
        if connectivity["unrouted_connections"] == 0:
            failures = {}
        else:
            failures = connectivity["unrouted_details"]
    except Exception as exc:
        stats["final_connectivity_audit_error"] = (
            f"{type(exc).__name__}: {exc}")

    stats["routing_budget_seconds"] = BOARD_TIME_BUDGET
    stats["routing_elapsed_seconds"] = round(
        time.monotonic() - route_started, 6)
    stats["final_failed_connections"] = failures
    drc_regressions = int(stats.get("new_drc_violations", 0) or 0)
    stats["partial"] = bool(failures or drc_regressions)
    final_result["failures"] = failures
    final_result["stats"] = stats
    final_result["status"] = "partial" if stats["partial"] else "complete"
    tmp_json = routes_json + ".final.tmp"
    with open(tmp_json, "w") as f:
        json.dump(final_result, f)
    os.replace(tmp_json, routes_json)
    print(f"  [LE] plane nets: {stats.get('zone_nets')} "
          f"({stats.get('stitched')} stitch vias, {stats.get('through_pads')} through-hole pads)")
    print(f"  [LE] trace pairs: {stats.get('pairs')}; failed nets: {len(failures)}/{stats.get('nets')}")
    if drc_regressions:
        print(f"  [LE] DRC regression: {drc_regressions} new violation(s) "
              f"by type {stats.get('new_drc_violations_by_type', {})}")
    for net, fails in list(failures.items())[:10]:
        print(f"  [LE]   {net}: unrouted {', '.join(fails[:6])}")
    try:
        os.remove(routes_json + ".geo")
    except OSError:
        pass
    return (not failures and not drc_regressions), stats


def run_lineexplore(output_dir, prefix, via_cost=2.0, drc=None, verbose=True) -> bool:
    """Route <prefix>.kicad_pcb in place with LineExplore.

    Returns True when every multi-pad net is connected.  A partial route is
    still saved at <prefix>.kicad_pcb and explicitly described by the routing
    result sidecar; it is inspectable but not manufacturing-ready.
    """
    print("\n=== [Step 6] LineExplore autorouter ===")
    board_file = os.path.join(output_dir, f"{prefix}.kicad_pcb")
    if not os.path.exists(board_file):
        print("  [LE] board file not found — nothing to route.")
        return False
    drc = _load_drc(output_dir, prefix, drc)
    started = time.perf_counter()

    backup = os.path.join(output_dir, f"{prefix}_unrouted.kicad_pcb")
    shutil.copy2(board_file, backup)

    try:
        ok, _stats = route_board_file(board_file, board_file, drc,
                                      via_cost=via_cost, verbose=verbose)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print(f"  [LE] router exception: {exc}")
        shutil.copy2(backup, board_file)
        with open(os.path.join(output_dir, f"{prefix}_routing_result.json"), "w") as f:
            json.dump({"router": "lineexplore", "status": "unrouted",
                       "manufacturing_ready": False, "error": str(exc),
                       "routing_seconds": round(time.perf_counter() - started, 6)}, f, indent=2)
        return False

    routes_path = board_file + ".routes.json"
    failures = {}
    try:
        with open(routes_path) as f:
            failures = json.load(f).get("failures") or {}
    except (OSError, ValueError):
        pass
    result = {
        "router": "lineexplore",
        "status": "complete" if ok else "partial",
        "manufacturing_ready": False,
        "requires_drc": True,
        "failed_nets": sorted(failures),
        "failed_connections": failures,
        "blocked_routes": (_stats.get("blocked_routes", []) +
                           _stats.get("escape_blocked_routes", [])),
        "stats": _stats,
        "routing_seconds": round(time.perf_counter() - started, 6),
    }
    with open(os.path.join(output_dir, f"{prefix}_routing_result.json"), "w") as f:
        json.dump(result, f, indent=2)

    if not ok:
        debug_copy = os.path.join(output_dir, f"{prefix}_le_debug.kicad_pcb")
        shutil.copy2(board_file, debug_copy)
        print(f"  [LE] routing PARTIAL — inspectable board retained at {board_file}; "
              "NOT manufacturing-ready.")
        return False
    print(f"  [LE] routed board saved: {board_file}")
    return True


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Standalone 3D LineExplore router")
    ap.add_argument("board", nargs="?", help=".kicad_pcb to route")
    ap.add_argument("--drc", help="drc config JSON file (defaults mirror the runner)")
    ap.add_argument("--via-cost", type=float, default=2.0)
    ap.add_argument("--out", help="output board (default <stem>_le_routed.kicad_pcb)")
    ap.add_argument("--geometry-stage", nargs=4,
                    metavar=("GEO", "ROUTES", "VIACOST", "DEADLINE"),
                    help=argparse.SUPPRESS)   # internal subprocess entry
    ap.add_argument("--postprocess-stage", nargs=3,
                    metavar=("GEO", "ROUTES", "VIACOST"),
                    help=argparse.SUPPRESS)   # internal subprocess entry
    ap.add_argument("--audit-stage", nargs=3, metavar=("BOARD", "GEO", "OUT"),
                    help=argparse.SUPPRESS)   # internal subprocess entry
    ap.add_argument("--apply-stage", nargs=4,
                    metavar=("BOARD", "OUT", "ROUTES", "DRC"),
                    help=argparse.SUPPRESS)   # internal subprocess entry
    args = ap.parse_args()

    if args.geometry_stage:
        return _geometry_stage(args.geometry_stage[0],
                               args.geometry_stage[1],
                               float(args.geometry_stage[2]),
                               float(args.geometry_stage[3]))
    if args.postprocess_stage:
        return _postprocess_stage(*args.postprocess_stage[:2],
                                  float(args.postprocess_stage[2]))
    if args.audit_stage:
        board_f, geo_f, out_f = args.audit_stage
        with open(geo_f) as f:
            geo = json.load(f)
        esc = audit_zone_pads(board_f, geo)
        with open(out_f, "w") as f:
            json.dump(esc, f)
        return 0
    if args.apply_stage:
        board_f, out_f, routes_f, drc_f = args.apply_stage
        with open(routes_f) as f:
            routes = json.load(f)
        with open(drc_f) as f:
            drc = json.load(f)
        n_tracks, n_vias = apply_routes(board_f, out_f, routes, drc)
        print(f"  [LE] applied {n_tracks} tracks + {n_vias} vias -> {out_f}")
        return 0
    if not args.board:
        ap.error("board path required")

    drc = dict(DEFAULT_DRC)
    if args.drc and os.path.exists(args.drc):
        with open(args.drc) as f:
            drc.update(json.load(f))

    out = args.out or os.path.splitext(args.board)[0] + "_le_routed.kicad_pcb"
    ok, _stats = route_board_file(args.board, out, drc, via_cost=args.via_cost)
    return 0 if ok else 2

