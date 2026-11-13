"""Apply LineExplore tracks and vias to an existing KiCad board."""

from kicad_host import import_pcbnew
from router_constants import NM
from routing_rules import rule_for_net


def _strip_tracks_text(board_file):
    """Copy <board_file> with all top-level copper track/via/arc items removed
    (textual, paren-balanced) and return the temp path.

    board.Remove(track) is NOT an option here: it corrupts swig's wrapping
    for the rest of the process (later pcbnew constructors segfault — the
    codegen-authored boards carry pre-drawn straight-line stubs, and removing
    them killed the whole apply stage with SIGSEGV/-11). The benchmark
    variant boards are unrouted, so they never hit this; netlist-pipeline
    boards always do.
    """
    with open(board_file, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    out, depth, skip = [], 0, False
    for line in lines:
        s = line.lstrip()
        if not skip and depth == 1 and (
                s.startswith("(segment") or s.startswith("(via")
                or s.startswith("(arc")):
            skip = True
        if not skip:
            out.append(line)
        depth += line.count("(") - line.count(")")
        if skip and depth <= 1:
            skip = False
    tmp = board_file + ".notracks.kicad_pcb"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    return tmp


def apply_routes(board_file, out_file, routes, drc):
    pcbnew = import_pcbnew()

    board = pcbnew.LoadBoard(_strip_tracks_text(board_file))
    if board is None:
        raise RuntimeError(f"could not load board {board_file}")

    n_tracks = n_vias = 0
    for item in routes.get("items", []):
        net = board.FindNet(item["net"])
        net_rule = rule_for_net(drc, item["net"])
        if item["type"] == "track":
            trace_w = int(max(
                float(item.get("width", net_rule.get("trace_width", 0.25))),
                float(net_rule.get("trace_width", 0.25)),
            ) * NM)
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(int(item["a"][0] * NM), int(item["a"][1] * NM)))
            t.SetEnd(pcbnew.VECTOR2I(int(item["b"][0] * NM), int(item["b"][1] * NM)))
            t.SetLayer(item["layer"])
            t.SetWidth(trace_w)
            if net is not None:
                t.SetNet(net)
            board.Add(t)
            n_tracks += 1
        elif item["type"] == "via":
            via_drill_mm = max(
                float(item.get("drill", net_rule.get("via_drill", 0.3))),
                float(net_rule.get("via_drill", 0.3)),
            )
            via_size_mm = max(
                float(item.get("size", net_rule.get("via_track_width", 0.6))),
                float(net_rule.get("via_track_width", 0.6)),
                via_drill_mm + 2.0 * float(net_rule.get("via_annular_ring", 0.15)),
            )
            via_drill = int(via_drill_mm * NM)
            via_size = int(via_size_mm * NM)
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(int(item["p"][0] * NM), int(item["p"][1] * NM)))
            v.SetWidth(via_size)
            v.SetDrill(via_drill)
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            try:
                v.SetLayerSet(pcbnew.LSET.AllCuMask())
            except Exception:
                pass
            if net is not None:
                v.SetNet(net)
            board.Add(v)
            n_vias += 1

    # zones were never touched — refill so stitching vias/planes connect
    try:
        zones = list(board.Zones())
    except AttributeError:
        zones = []
    if zones:
        try:
            pcbnew.ZONE_FILLER(board).Fill(zones)
            print(f"  [LE] refilled {len(zones)} zone(s).")
        except Exception as exc:
            print(f"  [LE] zone refill failed ({exc}) — kicad-cli DRC will refill.")

    # Audit CANNOT run here: board.Remove() above corrupts swig's object
    # wrapping for the rest of this process (later GetConnectivity/LoadBoard
    # calls return raw SwigPyObject with no methods). The audit runs in a
    # fresh interpreter instead — see _audit_subprocess.
    pcbnew.SaveBoard(out_file, board)
    return n_tracks, n_vias
