"""KiCad board geometry extraction and board-rule clamping.

This phase reads an existing placed board; it does not change placement.
"""

import math

from kicad_host import import_pcbnew
from routing_rules import normalize_routing_rules
from router_constants import DEFAULT_DRC, NM


def _board_routing_floors(board):
    """Return the smallest routing dimensions allowed by this KiCad board."""
    def mm(value):
        return value / NM

    try:
        settings = board.GetDesignSettings()
        floors = {
            "trace_width": mm(settings.m_TrackMinWidth),
            "clearance": mm(settings.m_MinClearance),
            "via_drill": mm(settings.m_MinThroughDrill),
            "via_track_width": mm(settings.m_ViasMinSize),
            "via_annular_ring": mm(settings.m_ViasMinAnnularWidth),
            "hole_clearance": max(mm(settings.m_HoleClearance),
                                   mm(settings.m_HoleToHoleMin)),
            "outline_clearance": mm(settings.m_CopperEdgeClearance),
        }
    except Exception:
        floors = {key: 0.0 for key in DEFAULT_DRC}

    try:
        netclass = board.GetDesignSettings().m_NetSettings.GetDefaultNetclass()
        floors["clearance"] = max(floors["clearance"], mm(netclass.GetClearance()))
        floors["trace_width"] = max(floors["trace_width"], mm(netclass.GetTrackWidth()))
        floors["via_track_width"] = max(floors["via_track_width"], mm(netclass.GetViaDiameter()))
        floors["via_drill"] = max(floors["via_drill"], mm(netclass.GetViaDrill()))
    except Exception:
        pass
    return floors


def _clamp_routing_rules(drc, floors):
    """Keep configured and per-net rules at or above KiCad board minima."""
    drc = normalize_routing_rules({**DEFAULT_DRC, **(drc or {})})

    def clamp(rule):
        for key in ("trace_width", "clearance", "via_drill",
                    "via_annular_ring", "hole_clearance", "outline_clearance"):
            rule[key] = max(float(rule.get(key, DEFAULT_DRC[key])), floors[key])
        rule["via_track_width"] = max(
            float(rule.get("via_track_width", DEFAULT_DRC["via_track_width"])),
            floors["via_track_width"],
            rule["via_drill"] + 2.0 * rule["via_annular_ring"],
        )

    clamp(drc)
    for net_class in drc.get("net_classes", []):
        clamp(net_class)
    return normalize_routing_rules(drc)


def _clamp_routing_rules_for_board(board_file, drc):
    """Resolve KiCad minima before routing, not only during geometry export."""
    pcbnew = import_pcbnew()
    board = pcbnew.LoadBoard(board_file)
    if board is None:
        raise RuntimeError(f"could not load board rules from {board_file}")
    return _clamp_routing_rules(drc, _board_routing_floors(board))


def extract_geometry(board_file, drc, verbose=True):
    """Load the board and return a JSON-serializable geometry dict.
    Layer ids are opaque pcbnew integers throughout."""
    pcbnew = import_pcbnew()
    drc = normalize_routing_rules({**DEFAULT_DRC, **(drc or {})})
    board = pcbnew.LoadBoard(board_file)
    if board is None:
        raise RuntimeError(f"could not load board {board_file}")

    n = board.GetCopperLayerCount()
    try:
        copper = [l for l in board.GetEnabledLayers().CuStack()]
        if len(copper) != n:
            raise RuntimeError
    except Exception:
        copper = ([pcbnew.F_Cu]
                  + [getattr(pcbnew, f"In{i}_Cu") for i in range(1, n - 1)]
                  + [pcbnew.B_Cu])

    def mm(v):
        return v / NM

    drc = _clamp_routing_rules(drc, _board_routing_floors(board))

    pads = []
    for fp in board.GetFootprints():
        try:
            ref = fp.GetReference()
        except Exception:
            ref = "?"
        for p in fp.Pads():
            net = (p.GetNetname() or "").strip()
            if not net:
                # Unconnected / NPTH pads are still copper and still drilled:
                # keep them as obstacles with a UNIQUE net name so no routing
                # net ever excludes them (a shared "" would merge them into
                # one phantom multi-pad net) — tracks ran straight over these
                # pads when they were skipped, shorting to them.
                net = f"__pad_{ref}_{len(pads)}"
            bb = p.GetBoundingBox()
            pos = p.GetPosition()
            try:
                through = bool(p.HasHole())
            except Exception:
                through = False
            # GetLayer() is USELESS here: for pads of a flipped (bottom-side)
            # footprint it still returns the F_Cu sentinel, so the entire
            # bottom side landed on TOP as phantom obstacles (a bottom 0805
            # 0.75mm from a top 0805 then blocks every escape). IsOnLayer()
            # is authoritative — and paste/mask-only apertures (F=False B=
            # False) have no copper at all: skip them entirely.
            if not through:
                layer = None
                try:
                    for cl in copper:
                        if p.IsOnLayer(cl):
                            layer = int(cl)
                            break
                except Exception:
                    layer = int(p.GetLayer())
                if layer is None:
                    continue
            else:
                layer = int(copper[0])
            hole = 0.0
            if through:
                try:
                    dsz = p.GetDrillSize()
                    hole = round(mm(max(dsz.x, dsz.y)) / 2.0, 6)  # hole RADIUS
                except Exception:
                    hole = 0.0
            # True pad size + orientation. GetBoundingBox() is the AXIS-ALIGNED
            # bbox: for a 45-degree-rotated 1.475x0.3 LQFP pad it is a 1.26mm
            # SQUARE, 4x too fat across the row — adjacent modeled pads then
            # wall off every fan-out escape (every net touching the part fails).
            try:
                sz = p.GetSize()
                pw, ph = mm(sz.x), mm(sz.y)
            except Exception:
                pw, ph = mm(bb.GetWidth()), mm(bb.GetHeight())
            try:
                ang = p.GetOrientation()          # EDA_ANGLE (kicad 9+) or float deg
                rot = float(ang.AsRadians())
            except AttributeError:
                try:
                    rot = math.radians(float(ang))
                except Exception:
                    rot = 0.0
            except Exception:
                rot = 0.0
            # Exact copper outline, board-absolute, rotation/custom-primitives
            # already baked in. GetBoundingBox()/GetSize() are both WRONG for
            # `smd custom` pads: the primitives (gr_poly tab etc.) extend
            # asymmetrically from the pad anchor, so a symmetric bbox at the
            # pad position fabricates copper — on simple_pixelboost U1's SOT-89
            # +12V tab grew 1.56 mm past its true edge and buried a neighbor
            # GND pad under a phantom obstacle. GetEffectivePolygon gives the
            # real shape (and rounds/chamfers exactly, for free).
            poly = None
            try:
                eps = p.GetEffectivePolygon(int(layer), 0)
                ring = eps.Outline(0)
                pts = [(round(mm(ring.CPoint(i).x), 3),
                        round(mm(ring.CPoint(i).y), 3))
                       for i in range(ring.PointCount())]
                if len(pts) >= 3:
                    poly = pts
            except Exception:
                poly = None
            hw = mm(bb.GetWidth()) / 2.0
            hh = mm(bb.GetHeight()) / 2.0
            rmax = math.hypot(hw, hh)
            if poly:
                xs = [q[0] for q in poly]
                ys = [q[1] for q in poly]
                hw = (max(xs) - min(xs)) / 2.0
                hh = (max(ys) - min(ys)) / 2.0
                # furthest vertex from the pad ANCHOR (not the bbox center):
                # escape/via probes must clear an off-center custom shape, and
                # half-extents do not (they assume the anchor is centered).
                rmax = max(math.hypot(q[0] - mm(pos.x), q[1] - mm(pos.y))
                           for q in poly)
            try:
                local_clearance = max(0.0, mm(p.GetLocalClearance()))
            except Exception:
                local_clearance = 0.0
            pads.append({"net": net, "ref": ref,
                         "x": round(mm(pos.x), 6), "y": round(mm(pos.y), 6),
                         "hw": round(hw, 6), "hh": round(hh, 6),
                         "w": round(pw, 6), "h": round(ph, 6),
                         "rot": round(rot, 8),
                         "through": through, "layer": int(layer),
                         "hole": hole, "poly": poly, "rmax": round(rmax, 6),
                         "local_clearance": round(local_clearance, 6)})
    if verbose:
        print(f"  [LE] extracted {len(pads)} pads on {n} copper layers")

    # zones: {net -> layer}; keep the objects out of the JSON (ids only)
    netcode_to_name = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            try:
                netcode_to_name[p.GetNetCode()] = p.GetNetname()
            except Exception:
                pass
    zone_map = {}
    zone_fills = []
    # Geometry-only metadata for the optional 135-degree post-process.  The
    # router's existing zone_map behavior remains unchanged; these records
    # only reserve enough room around thermally connected pads that moving a
    # foreign trace cannot starve a zone spoke during the later refill.
    zone_thermals = []
    try:
        zones = list(board.Zones())
    except AttributeError:
        try:
            zones = list(board.GetZones() or [])
        except AttributeError:
            zones = []
    for z in zones:
        try:
            name = z.GetNet().GetNetname()
        except Exception:
            name = netcode_to_name.get(z.GetNetCode(), "")
        # Filled copper is a real obstacle for foreign nets. The previous
        # router used only zone ownership for connectivity and could route
        # across an existing pour, leaving isolated islands after refill.
        try:
            for layer in z.GetLayerSet().Seq():
                layer = int(layer)
                if layer not in [int(c) for c in copper] or not z.HasFilledPolysForLayer(layer):
                    continue
                polyset = z.GetFilledPolysList(layer)
                for outline_index in range(polyset.OutlineCount()):
                    outline = polyset.Outline(outline_index)
                    outer = [(round(mm(outline.CPoint(i).x), 6),
                              round(mm(outline.CPoint(i).y), 6))
                             for i in range(outline.PointCount())]
                    if len(outer) < 3:
                        continue
                    holes = []
                    for hole_index in range(polyset.HoleCount(outline_index)):
                        hole = polyset.Hole(outline_index, hole_index)
                        points = [(round(mm(hole.CPoint(i).x), 6),
                                   round(mm(hole.CPoint(i).y), 6))
                                  for i in range(hole.PointCount())]
                        if len(points) >= 3:
                            holes.append(points)
                    zone_fills.append({"layer": layer, "net": name,
                                       "poly": outer, "holes": holes})
                    # A multilayer zone may report GetLayer() == -1. Select
                    # only a real copper layer that has filled polygons so
                    # the stitch pass never targets a sentinel layer.
                    if name and name not in zone_map:
                        zone_map[name] = layer
        except Exception:
            # Empty or unsupported fills remain governed by their zone
            # outline and the final KiCad refill/DRC audit.
            pass
        if name:
            try:
                connection = int(z.GetPadConnection())
            except Exception:
                connection = 1
            try:
                thermal_gap = round(mm(z.GetThermalReliefGap()), 6)
            except Exception:
                thermal_gap = 0.5
            try:
                spoke_width = round(mm(z.GetThermalReliefSpokeWidth()), 6)
            except Exception:
                spoke_width = 0.5
            zone_thermals.append({"net": name, "layer": int(z.GetLayer()),
                                  "connection": connection,
                                  "gap": thermal_gap,
                                  "spoke_width": spoke_width})
    if verbose:
        print(f"  [LE] zone nets: {zone_map}")

    bb = board.GetBoardEdgesBoundingBox()
    if bb.GetWidth() > 0 and bb.GetHeight() > 0:
        x1, y1 = mm(bb.GetX()), mm(bb.GetY())
        x2, y2 = x1 + mm(bb.GetWidth()), y1 + mm(bb.GetHeight())
    else:
        xs = [p["x"] for p in pads] or [0.0]
        ys = [p["y"] for p in pads] or [0.0]
        x1, y1, x2, y2 = min(xs) - 5, min(ys) - 5, max(xs) + 5, max(ys) + 5
    # A board whose Edge.Cuts covers only part of the design (the XESC
    # benchmark outlines just its power panel, its controller sits 100mm
    # away with no outline) would leave real pads OUTSIDE the keep-in —
    # every net there fails containment. Trust the pads: expand to the pad
    # hull when anything falls outside the drawn edge.
    if pads:
        tol = 0.5
        if (min(p["x"] for p in pads) < x1 - tol or max(p["x"] for p in pads) > x2 + tol
                or min(p["y"] for p in pads) < y1 - tol
                or max(p["y"] for p in pads) > y2 + tol):
            x1 = min(x1, min(p["x"] for p in pads) - 2.0)
            y1 = min(y1, min(p["y"] for p in pads) - 2.0)
            x2 = max(x2, max(p["x"] for p in pads) + 2.0)
            y2 = max(y2, max(p["y"] for p in pads) + 2.0)
            if verbose:
                print("  [LE] WARNING: pads outside Edge.Cuts — keep-in "
                      "expanded to pad hull")

    # Edge.Cuts SEGMENTS, as exact obstacles (net=None -> never excludable).
    # The keep-in box above is a coarse bound and, on a board with a PARTIAL
    # outline (XESC draws only its power panel; the keep-in then expands to
    # the pad hull), it happily lets tracks cut the corner of the drawn edge —
    # kicad-cli flags them copper_edge_clearance. Blocking the real segments
    # on every copper layer is exact for partial and full outlines alike.
    edges = []
    copper_graphics = []

    def add_copper_graphic(item, ref=None):
        try:
            layer = int(item.GetLayer())
            if layer not in [int(c) for c in copper]:
                return
            polyset = pcbnew.SHAPE_POLY_SET()
            # Convert every copper graphic shape to its effective copper
            # geometry, including line width, arcs and filled polygons.
            item.TransformShapeToPolySet(polyset, layer, 0, 10_000, 0)
            for outline_index in range(polyset.OutlineCount()):
                outline = polyset.Outline(outline_index)
                outer = [(round(mm(outline.CPoint(i).x), 6),
                          round(mm(outline.CPoint(i).y), 6))
                         for i in range(outline.PointCount())]
                if len(outer) < 3:
                    continue
                holes = []
                for hole_index in range(polyset.HoleCount(outline_index)):
                    hole = polyset.Hole(outline_index, hole_index)
                    points = [(round(mm(hole.CPoint(i).x), 6),
                               round(mm(hole.CPoint(i).y), 6))
                              for i in range(hole.PointCount())]
                    if len(points) >= 3:
                        holes.append(points)
                copper_graphics.append({"layer": layer, "net": None,
                                        "poly": outer, "holes": holes,
                                        "ref": ref})
        except Exception:
            return

    # Footprint copper (logos, custom copper marks, and other fp graphics) is
    # unnetted unless explicitly represented by pads. Treat it as a permanent
    # obstacle so a routed track cannot short it.
    for fp in board.GetFootprints():
        try:
            ref = fp.GetReference()
            for item in fp.GraphicalItems():
                add_copper_graphic(item, ref)
        except Exception:
            continue

    try:
        for it in board.GetDrawings():
            if it.Type() != pcbnew.PCB_SHAPE_T:
                continue
            try:
                if int(it.GetLayer()) in [int(c) for c in copper]:
                    add_copper_graphic(it)
                    continue
                if it.GetLayer() != pcbnew.Edge_Cuts:
                    continue
                if it.GetShape() != pcbnew.S_SEGMENT:
                    continue
                s, e = it.GetStart(), it.GetEnd()
                if s != e:
                    edges.append([round(mm(s.x), 3), round(mm(s.y), 3),
                                  round(mm(e.x), 3), round(mm(e.y), 3)])
            except Exception:
                pass
    except Exception:
        pass

    return {"copper": [int(l) for l in copper], "drc": drc,
            "pads": pads, "zone_map": zone_map,
            "zone_thermals": zone_thermals, "edges": edges,
            "copper_graphics": copper_graphics, "zone_fills": zone_fills,
            "edge": [x1, y1, x2, y2]}
