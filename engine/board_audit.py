"""KiCad connectivity, zone, and native DRC audit helpers."""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

from board_apply import _strip_tracks_text

WORKER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lineexplore.py")


def _audit_zone_pads_on(board, geo):
    """Post-refill connectivity audit on a LIVE board: which zone-net pads
    did the refill actually reach? `pad.layer == zone_layer -> refill
    connects it` is only a hope — other nets' routed copper can carve a
    pocket around an SMD pad and the fill never touches it (DRC then reports
    pad<->zone unconnected).

    Uses pcbnew's own connectivity graph (the same thing DRC uses). Returns
    escape_pads: the unreached pads plus, per pad, the nearest same-net pad
    the graph DOES reach, as the anchor an escape trace can aim at."""
    zone_nets = set(geo.get("zone_map") or {})
    if not zone_nets:
        return []
    geopads = {(d["ref"], d["x"], d["y"]): d for d in geo["pads"]}
    conn = board.GetConnectivity()
    conn.RecalculateRatsnest()
    good, bad = [], []
    geopad_objects = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            name = p.GetNetname()
            if name not in zone_nets:
                continue
            key = (fp.GetReference(), round(p.GetPosition().x / 1e6, 6),
                   round(p.GetPosition().y / 1e6, 6))
            d = geopads.get(key)
            if d is None:
                continue
            geopad_objects[key] = p
            try:
                ok = any(conn.IsConnectedOnLayer(p, int(l))
                         for l in geo["copper"])
            except Exception:
                ok = True      # API surprise: never claim a pad is broken
            (good if ok else bad).append(d)
    # Keep one representative for every physically connected component,
    # including each already-connected anchor component. Routing one anchor
    # for the whole net can leave other disconnected copper trees behind;
    # routing every pad wastes searches on pads that already share copper.
    def component_groups(records):
        groups = {}
        for d in records:
            item_key = (d["ref"], d["x"], d["y"])
            pad = geopad_objects.get(item_key)
            component = None
            if pad is not None:
                try:
                    items = conn.GetConnectedItems(pad)
                    # Use the underlying C++ pointer, not the temporary SWIG
                    # wrapper address, to identify shared connected items.
                    component = frozenset(int(item.this) for item in items)
                except Exception:
                    component = None
            key = (d["net"], component) if component else (d["net"], item_key)
            groups.setdefault(key, []).append(d)
        return groups

    good_groups = component_groups(good)
    bad_groups = component_groups(bad)
    good_representatives = {}
    bad_representatives = {}
    for net in {d["net"] for d in good} | {d["net"] for d in bad}:
        good_groups_for_net = [group for (name, _), group in good_groups.items()
                               if name == net]
        bad_groups_for_net = [group for (name, _), group in bad_groups.items()
                              if name == net]
        all_good = [d for group in good_groups_for_net for d in group]
        all_bad = [d for group in bad_groups_for_net for d in group]
        cx = (sum(d["x"] for d in all_bad) / len(all_bad)
              if all_bad else 0.0)
        cy = (sum(d["y"] for d in all_bad) / len(all_bad)
              if all_bad else 0.0)
        # Route every disconnected pad group toward one stable point already
        # connected to the net. Sending all connected-component anchors into
        # the escape search multiplies paths through dense pours and previously
        # worsened completion on the general benchmark boards. Prefer a
        # through-hole pad when available because it can connect across layers.
        good_representatives[net] = []
        if all_good:
            candidates = [d for d in all_good if d["through"]] or all_good
            good_representatives[net].append(min(
                candidates,
                key=lambda d: math.hypot(d["x"] - cx, d["y"] - cy)))
        if not bad_groups_for_net:
            continue
        bad_representatives[net] = []
        for group in bad_groups_for_net:
            if all_good:
                ax = sum(d["x"] for d in all_good) / len(all_good)
                ay = sum(d["y"] for d in all_good) / len(all_good)
                representative = min(
                    group,
                    key=lambda d: math.hypot(d["x"] - ax, d["y"] - ay))
            else:
                representative = group[0]
            bad_representatives[net].append(representative)

    esc, seen = [], set()

    def add(d, role):
        k = (d["net"], d["ref"], d["x"], d["y"])
        if k not in seen:
            seen.add(k)
            esc.append({**d, "escape_role": role})

    nets_with_bad_components = set(bad_representatives)
    for net in sorted(nets_with_bad_components):
        for d in good_representatives.get(net, []):
            add(d, "anchor")
    for net in sorted(nets_with_bad_components):
        for d in bad_representatives[net]:
            add(d, "unconnected")
    return esc


def audit_zone_pads(board_file, geo):
    """Standalone wrapper: fresh process => LoadBoard is safe here."""
    sys.path.insert(0, '/usr/lib/python3/dist-packages')
    import pcbnew
    board = pcbnew.LoadBoard(board_file)
    if board is None:
        return []
    return _audit_zone_pads_on(board, geo)


def _audit_subprocess(board_file, geo):
    """Run audit_zone_pads in a fresh interpreter. The apply process has
    called board.Remove() by then, which corrupts swig's wrapping for every
    later object it returns (raw SwigPyObject, no methods) — the audit must
    not live there."""
    here = os.path.dirname(os.path.abspath(__file__))
    board_file = os.path.abspath(board_file)
    geo_json = board_file + ".auditgeo.json"
    out_json = board_file + ".audit.json"
    with open(geo_json, "w") as f:
        json.dump(geo, f)
    try:
        os.remove(out_json)
    except OSError:
        pass
    env = {k: v for k, v in os.environ.items()
           if k in ("PATH", "HOME", "LD_LIBRARY_PATH", "LANG")}
    proc = subprocess.run(
        [sys.executable, "-u", WORKER_SCRIPT,
         "--audit-stage", board_file, geo_json, out_json],
        env=env, cwd=here)
    if os.path.exists(out_json):
        with open(out_json) as f:
            return json.load(f)
    raise RuntimeError(f"audit stage exited {proc.returncode}")


def _native_kicad_drc(board_file):
    """Return a refilled native KiCad DRC report for a candidate board."""
    with tempfile.NamedTemporaryFile(prefix="lineexplore-candidate-drc-",
                                     suffix=".json", delete=False) as report:
        report_path = report.name
    try:
        proc = subprocess.run(
            ["kicad-cli", "pcb", "drc", "--format", "json",
             "--severity-all", "--refill-zones", "-o", report_path,
             os.path.abspath(board_file)],
            capture_output=True, text=True, timeout=120,
        )
        if proc.returncode not in (0, 5) or not os.path.exists(report_path):
            detail = (proc.stderr or proc.stdout or
                      f"kicad-cli exited {proc.returncode}").strip()
            raise RuntimeError(detail)
        with open(report_path) as source:
            return json.load(source)
    finally:
        try:
            os.remove(report_path)
        except OSError:
            pass


def _violation_identity(violation):
    items = []
    for item in violation.get("items") or []:
        pos = item.get("pos") or {}
        items.append((
            str(item.get("description") or ""),
            round(float(pos.get("x", 0.0)), 4),
            round(float(pos.get("y", 0.0)), 4),
        ))
    return (
        str(violation.get("type") or ""),
        str(violation.get("severity") or ""),
        str(violation.get("description") or ""),
        tuple(sorted(items)),
    )


def _new_drc_findings(before, after, *, severity=None, kind=None):
    """Count native DRC findings in ``after`` absent from ``before``."""
    existing = {}
    for violation in before.get("violations") or []:
        key = _violation_identity(violation)
        existing[key] = existing.get(key, 0) + 1
    added = []
    for violation in after.get("violations") or []:
        key = _violation_identity(violation)
        if severity and key[1] != severity:
            continue
        if kind and key[0] != kind:
            continue
        if existing.get(key, 0):
            existing[key] -= 1
        else:
            added.append(violation)
    return added


def _project_for_board(source_board):
    """Find the source project's rules for a staged PCB copy.

    Benchmark inputs commonly use ``_placement`` or ``_unrouted`` suffixes,
    while generated route attempts leave additional project files beside
    them. Prefer a project matching the normalized source board name before
    falling back to a unique project in the directory.
    """
    source_dir = os.path.dirname(source_board)
    source_stem = os.path.splitext(os.path.basename(source_board))[0]
    suffixes = ("_unrouted", "_placement", "_notracks", ".notracks")
    stems = [source_stem]
    normalized = source_stem
    changed = True
    while changed:
        changed = False
        for suffix in suffixes:
            if normalized.endswith(suffix):
                normalized = normalized[:-len(suffix)]
                stems.append(normalized)
                changed = True
                break
    for stem in stems:
        candidate = os.path.join(source_dir, stem + ".kicad_pro")
        if os.path.isfile(candidate):
            return candidate
    projects = sorted(name for name in os.listdir(source_dir)
                      if name.endswith(".kicad_pro"))
    if len(projects) == 1:
        return os.path.join(source_dir, projects[0])
    return None


def _audit_final_connectivity(board_file, source_board, report_override=None):
    """Use KiCad DRC for connectivity and report violations added by routing."""
    source_dir = os.path.dirname(source_board)
    project = _project_for_board(source_board)
    source_stem = (os.path.splitext(os.path.basename(project))[0]
                   if project else
                   os.path.splitext(os.path.basename(source_board))[0])
    with tempfile.TemporaryDirectory(prefix="lineexplore-audit-") as temp_dir:
        audit_stem = source_stem
        if project:
            shutil.copy2(project,
                         os.path.join(temp_dir, audit_stem + ".kicad_pro"))
        audit_board = os.path.join(temp_dir, audit_stem + ".kicad_pcb")
        shutil.copy2(board_file, audit_board)
        report_path = os.path.join(temp_dir, "drc.json")
        baseline_source = _strip_tracks_text(source_board)
        baseline_dir = os.path.join(temp_dir, "baseline")
        os.makedirs(baseline_dir)
        if project:
            shutil.copy2(project,
                         os.path.join(baseline_dir, audit_stem + ".kicad_pro"))
        baseline_board = os.path.join(baseline_dir, audit_stem + ".kicad_pcb")
        shutil.copy2(baseline_source, baseline_board)

        def run_drc(path, output):
            proc = subprocess.run(
                ["kicad-cli", "pcb", "drc", "--format", "json",
                 "--severity-all", "--refill-zones", "-o", output, path],
                capture_output=True, text=True)
            if proc.returncode not in (0, 5) or not os.path.exists(output):
                detail = (proc.stderr or proc.stdout or
                          f"kicad-cli exited {proc.returncode}").strip()
                raise RuntimeError(detail)
            with open(output) as f:
                return json.load(f)

        baseline_report = run_drc(
            baseline_board, os.path.join(temp_dir, "baseline-drc.json"))
        # The final-shape gate already ran native DRC against the exact board
        # we keep. Reuse that report instead of refilling and checking the same
        # board a second time during final connectivity accounting.
        report = (report_override if report_override is not None else
                  run_drc(audit_board, report_path))
    unconnected = report.get("unconnected_items") or []
    net_open_items = {}
    for index, item in enumerate(unconnected, 1):
        endpoints = [str(endpoint.get("description") or "")
                     for endpoint in item.get("items") or []]
        names = set()
        for desc in endpoints:
            if "[" in desc and "]" in desc:
                names.add(desc.rsplit("[", 1)[-1].split("]", 1)[0])
        label = " <-> ".join(endpoints) or f"open_connection_{index}"
        for net in names or {"<unknown>"}:
            net_open_items.setdefault(net, []).append(label)

    def violation_key(violation):
        # UUIDs are unstable for newly emitted tracks. Use the human-readable
        # rule and item geometry so pre-existing findings can be subtracted.
        items = []
        for item in violation.get("items") or []:
            pos = item.get("pos") or {}
            items.append((
                str(item.get("description") or ""),
                round(float(pos.get("x", 0.0)), 4),
                round(float(pos.get("y", 0.0)), 4),
            ))
        return (
            str(violation.get("type") or ""),
            str(violation.get("severity") or ""),
            str(violation.get("description") or ""),
            tuple(sorted(items)),
        )

    before = {}
    for item in baseline_report.get("violations") or []:
        key = violation_key(item)
        before[key] = before.get(key, 0) + 1
    added_by_type = {}
    for item in report.get("violations") or []:
        key = violation_key(item)
        if before.get(key, 0):
            before[key] -= 1
        else:
            kind = key[0] or "unknown"
            added_by_type[kind] = added_by_type.get(kind, 0) + 1
    added_count = sum(added_by_type.values())
    return {
        "unrouted_connections": len(unconnected),
        "unrouted_nets": sorted(net_open_items),
        "unrouted_details": net_open_items,
        "drc_violations": len(report.get("violations") or []),
        "baseline_drc_violations": len(baseline_report.get("violations") or []),
        "new_drc_violations": added_count,
        "new_drc_violations_by_type": added_by_type,
    }
