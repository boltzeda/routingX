"""Core geometry and routing state models used by LineExplore search."""

import math

from shapely.geometry import Point, LineString, box, Polygon
from shapely import affinity
from shapely.strtree import STRtree

from router_constants import *  # noqa: F403
from routing_rules import rule_for_net


class Obstacles:
    """Per-layer obstacle sets with STRtrees, rebuilt once per routed net.

    Three geometries per item:
      raw    — exact pad/track/via body (mm); a via disc (radius via_r +
               clearance + SAFETY) may not touch it (via pads exist on every
               layer).
      wire   — raw inflated by clearance + trace_width/2 + SAFETY (mitre
               join); a trace CENTERLINE may not touch it, which enforces
               edge-to-edge clearance with router safety on top.
      corner — raw inflated slightly PAST the wire margin (paper stage 1);
               its vertices are the A* waypoints. They must sit strictly
               OUTSIDE the wire poly: `intersects` is closed, so a waypoint
               even 10 um inside the wire buffer makes every segment that
               ends on it invalid and the A* can never leave the pads
               (found the hard way: isolation 33/44 -> 13/44).
    Items carry their net so the net being routed excludes its own geometry.
    """

    def __init__(self, clearance, trace_w):
        self.clearance = clearance
        self.trace_w = trace_w
        self.margin = clearance + trace_w / 2.0 + SAFETY
        # margin + half a safety: strictly outside the wire buffer (mitre
        # buffers of a convex body nest strictly), yet close enough to hug.
        self.corner_margin = clearance + trace_w / 2.0 + 1.5 * SAFETY
        self.entries = {}        # layer -> (raw, wire, corner, net, local clearance)
        self._trees = {}         # layer_id -> (wire_tree, raw_tree, keep_idx, mids)
        self._pts_cache = {}     # layer_id -> (n_entries, [waypoint coords])
        self._wgrid_cache = {}   # layer_id -> (n_entries, cells, bboxes, nets)
        self._current_net = None
        self.capture_blockers = False
        self.blocker_hits = {}

    def add(self, layer, raw_poly, net, local_clearance=0.0):
        local_clearance = max(0.0, float(local_clearance))
        extra = max(0.0, local_clearance - self.clearance)
        wire = raw_poly.buffer(self.margin + extra, join_style=2)
        # Corner waypoints: the TRUE inflated outline for polygonal obstacles
        # (pads — few vertices; the paper's stage-1 nodes). The bbox shortcut
        # puts a 45-degree pad's corners in the wrong places entirely and the
        # fan-out corridors between rotated pads get no node at their mouth.
        # Smooth shapes (via discs, buffered tracks) keep the bbox: a mitre
        # buffer of a disc sprays ~33 useless vertices per via.
        try:
            nvert = len(raw_poly.exterior.coords)
        except AttributeError:
            nvert = 99
        if nvert <= 8:
            corner = raw_poly.buffer(self.corner_margin + extra, join_style=2)
        else:
            corner = box(*raw_poly.bounds).buffer(self.corner_margin + extra,
                                                  join_style=2)
        self.entries.setdefault(layer, []).append(
            (raw_poly, wire, corner, net, local_clearance))
        self._trees.pop(layer, None)   # lazily rebuilt by _ensure()
        self._pts_cache.pop(layer, None)
        self._wgrid_cache.pop(layer, None)

    def set_rule(self, clearance, trace_w):
        """Reinflate raw copper for the net currently being searched."""
        clearance, trace_w = float(clearance), float(trace_w)
        if abs(clearance - self.clearance) < EPS and abs(trace_w - self.trace_w) < EPS:
            return
        self.clearance = clearance
        self.trace_w = trace_w
        self.margin = clearance + trace_w / 2.0 + SAFETY
        self.corner_margin = clearance + trace_w / 2.0 + 1.5 * SAFETY
        rebuilt = {}
        for layer, entries in self.entries.items():
            for raw, _wire, _corner, net, local_clearance in entries:
                extra = max(0.0, local_clearance - self.clearance)
                wire = raw.buffer(self.margin + extra, join_style=2)
                try:
                    nvert = len(raw.exterior.coords)
                except AttributeError:
                    nvert = 99
                if nvert <= 8:
                    corner = raw.buffer(self.corner_margin + extra, join_style=2)
                else:
                    corner = box(*raw.bounds).buffer(
                        self.corner_margin + extra, join_style=2)
                rebuilt.setdefault(layer, []).append(
                    (raw, wire, corner, net, local_clearance))
        self.entries = rebuilt
        self._trees = {}
        self._pts_cache = {}
        self._wgrid_cache = {}

    def rebuild_for_net(self, net):
        self._current_net = net
        self._trees = {}

    def _ensure(self, layer):
        """Layer trees, rebuilt lazily after add() invalidation. Never treat
        a stale/missing tree as 'clear' — that is how shorts leak in."""
        t = self._trees.get(layer)
        if t is None:
            entries = self.entries.get(layer, [])
            keep = [i for i, e in enumerate(entries) if e[3] != self._current_net]
            wire = [entries[i][1] for i in keep]
            raw = [entries[i][0].buffer(max(
                0.0, entries[i][4] - self.clearance), join_style=2)
                   for i in keep]
            t = (STRtree(wire) if wire else None,
                 STRtree(raw) if raw else None, keep, None)
            self._trees[layer] = t
        return t

    def layer_points(self, layer):
        """Cached per-layer waypoint coords over ALL entries: every obstacle's
        corner-poly bbox vertices + gap midpoints (center of the corridor
        between two obstacle bboxes closer than GAP_PROBE mm — the node inside
        a tight corridor that no obstacle's own corners provide).

        Pure float math on bboxes (pads are axis-aligned rects; tracks/vias
        use the same bbox approximation the corner polys already use), so the
        per-net recompute is cheap. Including the current net's own obstacles
        is a harmless superset: segments are still validated by wire_clear,
        which excludes them. Returns [] when the layer has no entries."""
        ent = self.entries.get(layer, [])
        c = self._pts_cache.get(layer)
        if c is not None and c[0] == len(ent):
            return c[1]
        pts = set()
        boxes = []
        for _raw, _wire, corner, _net, _local_clearance in ent:
            x1, y1, x2, y2 = corner.bounds
            boxes.append((x1, y1, x2, y2))
            # BOTH point families: the bbox corners (diagonal fan-out
            # targets that carry a lot of routing on their own) and the true
            # inflated-outline vertices (the paper's stage-1 nodes — the only
            # ones sitting at a rotated pad's corridor mouths).
            pts.update(((x1, y1), (x2, y1), (x2, y2), (x1, y2)))
            try:
                pts.update((round(x, 3), round(y, 3))
                           for x, y in corner.exterior.coords)
            except AttributeError:
                pass
        # bbox pair gap midpoints, pruned by a coarse grid
        grid = {}
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            for gx in range(int(x1 / GC), int(x2 / GC) + 1):
                for gy in range(int(y1 / GC), int(y2 / GC) + 1):
                    grid.setdefault((gx, gy), []).append(i)
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            near = set()
            for gx in range(int(x1 / GC) - 1, int(x2 / GC) + 2):
                for gy in range(int(y1 / GC) - 1, int(y2 / GC) + 2):
                    for j in grid.get((gx, gy), ()):
                        if j > i:
                            near.add(j)
            for j in near:
                bx1, by1, bx2, by2 = boxes[j]
                dx = max(bx1 - x2, x1 - bx2, 0.0)
                dy = max(by1 - y2, y1 - by2, 0.0)
                if dx > GAP_PROBE or dy > GAP_PROBE or (dx <= 0.0 and dy <= 0.0):
                    continue
                if dx > 0.0 and dy > 0.0:      # diagonal corner gap
                    mx = (x2 + bx1) / 2.0 if bx1 >= x2 else (x1 + bx2) / 2.0
                    my = (y2 + by1) / 2.0 if by1 >= y2 else (y1 + by2) / 2.0
                elif dx > 0.0:                 # horizontal corridor
                    mx = (x2 + bx1) / 2.0 if bx1 >= x2 else (x1 + bx2) / 2.0
                    lo, hi = max(y1, by1), min(y2, by2)
                    my = (lo + hi) / 2.0
                else:                          # vertical corridor
                    my = (y2 + by1) / 2.0 if by1 >= y2 else (y1 + by2) / 2.0
                    lo, hi = max(x1, bx1), min(x2, bx2)
                    mx = (lo + hi) / 2.0
                pts.add((round(mx, 3), round(my, 3)))
        out = [(round(x, 3), round(y, 3)) for x, y in pts]
        self._pts_cache[layer] = (len(ent), out)
        return out

    def _wire_grid(self, layer):
        """Per-layer uniform grid of WIRE bounding boxes over all entries,
        cached by entry count. Pure-float overlap prefilter for wire_clear:
        if no wire bbox even overlaps the segment bbox, the segment is
        provably clear and no shapely object needs to exist (this check runs
        tens of millions of times on dense boards)."""
        ent = self.entries.get(layer, [])
        c = self._wgrid_cache.get(layer)
        if c is not None and c[0] == len(ent):
            return c
        cells = {}
        bboxes = []
        nets = []
        for i, (_raw, wire, _corner, net, _local_clearance) in enumerate(ent):
            x1, y1, x2, y2 = wire.bounds
            bboxes.append((x1, y1, x2, y2))
            nets.append(net)
            for gx in range(int(x1 / WCELL), int(x2 / WCELL) + 1):
                for gy in range(int(y1 / WCELL), int(y2 / WCELL) + 1):
                    cells.setdefault((gx, gy), []).append(i)
        c = (len(ent), cells, bboxes, nets)
        self._wgrid_cache[layer] = c
        return c

    def wire_clear(self, layer, a, b):
        _n, cells, bboxes, nets = self._wire_grid(layer)
        if not bboxes:
            return True
        if a[0] <= b[0]:
            sx1, sx2 = a[0], b[0]
        else:
            sx1, sx2 = b[0], a[0]
        if a[1] <= b[1]:
            sy1, sy2 = a[1], b[1]
        else:
            sy1, sy2 = b[1], a[1]
        cur = self._current_net
        gx0, gx1 = int(sx1 / WCELL), int(sx2 / WCELL)
        gy0, gy1 = int(sy1 / WCELL), int(sy2 / WCELL)
        for gx in range(gx0, gx1 + 1):
            for gy in range(gy0, gy1 + 1):
                for i in cells.get((gx, gy), ()):
                    if nets[i] == cur:
                        continue
                    bx1, by1, bx2, by2 = bboxes[i]
                    if bx1 <= sx2 and sx1 <= bx2 and by1 <= sy2 and sy1 <= by2:
                        # bbox overlap is only necessary, not sufficient:
                        # settle it with the exact prepared-geometry query
                        t = self._ensure(layer)
                        if t[0] is None:
                            return True
                        matches = t[0].query(
                            LineString([a, b]), predicate="intersects")
                        if len(matches) and self.capture_blockers:
                            hit_nets = set()
                            for tree_index in matches:
                                entry_index = t[2][int(tree_index)]
                                blocker = self.entries[layer][entry_index][3]
                                if blocker is not None and blocker != cur:
                                    hit_nets.add(blocker)
                            for blocker in hit_nets:
                                self.blocker_hits[blocker] = (
                                    self.blocker_hits.get(blocker, 0) + 1)
                        return len(matches) == 0
        return True

    def via_clear(self, copper, x, y, via_r):
        disc = Point(x, y).buffer(via_r + self.clearance + SAFETY)
        for layer in copper:
            t = self._ensure(layer)
            if t[1] is None:
                continue
            matches = t[1].query(disc, predicate="intersects")
            if len(matches) > 0:
                if self.capture_blockers:
                    hit_nets = set()
                    for tree_index in matches:
                        entry_index = t[2][int(tree_index)]
                        blocker = self.entries[layer][entry_index][3]
                        if blocker is not None and blocker != self._current_net:
                            hit_nets.add(blocker)
                    for blocker in hit_nets:
                        self.blocker_hits[blocker] = (
                            self.blocker_hits.get(blocker, 0) + 1)
                return False
        return True


class Pad:
    __slots__ = ("net", "x", "y", "hw", "hh", "through", "layer", "ref", "hole",
                 "w", "h", "rot", "_rect", "rmax", "poly", "local_clearance")

    def __init__(self, d):
        self.net, self.ref = d["net"], d.get("ref", "?")
        self.x, self.y = d["x"], d["y"]
        self.hw, self.hh = d["hw"], d["hh"]
        self.through, self.layer = d["through"], d["layer"]
        self.hole = float(d.get("hole", 0.0))
        self.w = float(d.get("w", 2.0 * self.hw))
        self.h = float(d.get("h", 2.0 * self.hh))
        self.rot = float(d.get("rot", 0.0))
        self._rect = None
        self.poly = d.get("poly")
        self.local_clearance = float(d.get("local_clearance", 0.0))
        rmax = float(d.get("rmax", 0.0))
        self.rmax = rmax if rmax > 0.0 else math.hypot(self.hw, self.hh)

    def rect(self):
        """True pad copper. Prefer the exact effective polygon extracted from
        pcbnew (custom/rotated/round shapes handled exactly); fall back to the
        rotated-rect approximation for legacy geo dicts without "poly"."""
        if self._rect is None:
            if self.poly:
                self._rect = Polygon(self.poly)
                if not self._rect.is_valid:
                    self._rect = self._rect.buffer(0)
            else:
                self._rect = self._fallback_rect()
        return self._rect

    def _fallback_rect(self):
        r = math.degrees(self.rot) % 90.0
        if r < 0.5 or r > 89.5 or self.w <= 0.0 or self.h <= 0.0:
            return box(self.x - self.hw, self.y - self.hh,
                       self.x + self.hw, self.y + self.hh)
        base = box(self.x - self.w / 2.0, self.y - self.h / 2.0,
                   self.x + self.w / 2.0, self.y + self.h / 2.0)
        # KiCad RotatePoint: x' = x.cos + y.sin, y' = -x.sin + y.cos
        # = shapely CCW rotation by NEGATIVE orientation.
        return affinity.rotate(
            base, -math.degrees(self.rot), origin=(self.x, self.y))

    def accessible(self, copper):
        return list(copper) if self.through else [self.layer]


class Env:
    """Geometry environment + emitted-item sink (pure shapely)."""

    def __init__(self, geo, via_cost=2.0):
        drc = {**DEFAULT_DRC, **(geo.get("drc") or {})}
        self.copper = geo["copper"]
        self.via_cost = via_cost
        self.drc = drc
        self.clearance = float(drc["clearance"])
        self.trace_w = float(drc["trace_width"])
        self.via_drill = float(drc["via_drill"])
        self.via_size = max(float(drc.get("via_track_width", 0.6)),
                            self.via_drill + 2.0 * float(drc.get("via_annular_ring", 0.15)))
        self.via_r = self.via_size / 2.0
        self.pads = [Pad(d) for d in geo["pads"]]
        self.zone_fills = {}
        for fill in geo.get("zone_fills") or []:
            try:
                zone_poly = Polygon(fill["poly"], fill.get("holes") or [])
                if not zone_poly.is_valid:
                    zone_poly = zone_poly.buffer(0)
                if not zone_poly.is_empty:
                    key = (fill["net"], int(fill["layer"]))
                    self.zone_fills.setdefault(key, []).append(zone_poly)
            except Exception:
                continue
        configured_penalty_nets = geo.get("zone_crossing_penalty_nets")
        if configured_penalty_nets is None:
            configured_penalty_nets = (
                set(geo.get("protected_zone_map") or {})
                | set(geo.get("zone_map") or {}))
        self.zone_crossing_penalty_nets = set(configured_penalty_nets)
        self.zone_fill_indexes = {}
        if self.zone_crossing_penalty_nets:
            by_layer = {}
            for (zone_net, layer), polygons in self.zone_fills.items():
                if zone_net not in self.zone_crossing_penalty_nets:
                    continue
                by_layer.setdefault(layer, []).extend(
                    (zone_net, polygon) for polygon in polygons)
            for layer, entries in by_layer.items():
                self.zone_fill_indexes[layer] = (
                    STRtree([polygon for _zone_net, polygon in entries]),
                    entries,
                )
        self._via_pts = set()      # emitted via centers (dedup)
        self.hole_clear = float(drc.get("hole_clearance", 0.25))
        # every drilled hole on the board (through pads at start, vias as they
        # are emitted): hole WALLS must stay hole_clearance apart even for the
        # SAME net — same-net obstacles are excluded from via_clear, so this
        # is the only hole-spacing guard. Tagged with the owning net so a
        # rip-up can withdraw its own vias; pad holes are tagged None (a pad
        # is never ripped, so its hole guard is permanent).
        self._holes = [(p.x, p.y, p.hole, None) for p in self.pads
                       if p.through and p.hole > 0.0]
        self.obst = Obstacles(self.clearance, self.trace_w)
        # Unnetted copper graphics are obstacles for every routed net.
        # Filled zones remain soft during path search so the router can use
        # inner layers; the native DRC candidate gate handles orphan islands
        # introduced by a route after KiCad refills the board.
        for item in geo.get("copper_graphics") or []:
            try:
                raw = Polygon(item["poly"], item.get("holes") or [])
                if not raw.is_valid:
                    raw = raw.buffer(0)
                if raw.is_empty:
                    continue
                self.obst.add(int(item["layer"]), raw, None)
            except Exception:
                continue
        self.items = []          # emitted routes: track/via dicts (mm)
        self.escape_mode = bool(geo.get("escape_pads"))
        self.preitem_nets = {
            it.get("net") for it in (geo.get("pre_items") or [])
            if it.get("net") is not None
        }
        self.search_stats = {
            "search_calls": 0, "search_expansions": 0,
            "adaptive_searches": 0, "adaptive_expansions": 0,
            "adaptive_hits": 0, "adaptive_bypassed": 0,
            "fullboard_fallbacks": 0,
            "parallel_path_candidates": 0, "parallel_path_wins": 0,
            "parallel_path_workers_peak": 0,
            "parallel_path_worker_errors": 0,
        }

        # Routes emitted by an EARLIER pass (zone-escape stage): solid
        # copper for this pass, and their drills are holes like any other.
        for it in (geo.get("pre_items") or []):
            if it.get("type") == "track" and it["a"] != it["b"]:
                prior_width = float(it.get("width", self.trace_w))
                self.obst.add(int(it["layer"]),
                              LineString([tuple(it["a"]), tuple(it["b"])])
                              .buffer(prior_width / 2.0), it["net"])
            elif it.get("type") == "via":
                px, py = float(it["p"][0]), float(it["p"][1])
                prior_drill = float(it.get("drill", self.via_drill))
                prior_size = float(it.get("size", self.via_size))
                self._holes.append((px, py, prior_drill / 2.0, it["net"]))
                self._via_pts.add((round(px, 3), round(py, 3)))
                disc = Point((px, py)).buffer(prior_size / 2.0)
                for layer in self.copper:
                    self.obst.add(layer, disc, it["net"])
            self.items.append(it)

        x1, y1, x2, y2 = geo["edge"]
        oc = float(drc.get("outline_clearance", 0.5))
        w = self.trace_w / 2.0 + SAFETY
        self.keep_wire = box(x1 + oc + w, y1 + oc + w, x2 - oc - w, y2 - oc - w)
        r = self.via_r + SAFETY
        self.keep_via = box(x1 + oc + r, y1 + oc + r, x2 - oc - r, y2 - oc - r)
        self.board_box = box(x1, y1, x2, y2)
        # Exact Edge.Cuts segments as permanent obstacles (net=None is never
        # the current net, so they can never be excluded). The Obstacles wire
        # inflation adds clearance + half trace + SAFETY on top of the
        # epsilon-thin segment rectangle, which is precisely the DRC rule.
        # This is what actually enforces copper_edge_clearance on boards with
        # a PARTIAL outline, where the keep-in box was widened to the pad hull
        # and no longer stands in for the drawn edge.
        for ex in (geo.get("edges") or []):
            # Obstacles.add inflates by `clearance` + half trace; the DRC edge
            # rule is `outline_clearance` + half trace, so pre-thicken by the
            # difference (they are equal on many boards, when oc<=clearance
            # the inflation is already the stricter of the two).
            configured_clearances = [float(drc["clearance"])]
            configured_clearances.extend(
                float(item.get("clearance", drc["clearance"]))
                for item in drc.get("net_classes", [])
            )
            # Raw edge thickening is based on the smallest class clearance;
            # later dynamic reinflation is therefore never weaker than the
            # requested copper-to-edge rule for a partial/non-rectangular edge.
            extra = max(0.0, oc - min(configured_clearances))
            erect = LineString([(ex[0], ex[1]), (ex[2], ex[3])]) \
                .buffer(extra + 0.001, cap_style=2)
            for layer in self.copper:
                self.obst.add(layer, erect, None)

    def inside_filled_zone(self, net, layer, point):
        """Whether a proposed stitch via lands in this net's filled copper."""
        return any(poly.covers(Point(point))
                   for poly in self.zone_fills.get((net, int(layer)), ()))

    def zone_crossing_cost(self, net, layer, a, b):
        """Softly prefer paths that do not cut through foreign-net pours.

        Filled zones stay traversable; the per-crossing cost lets A* choose a
        modest detour when one is available without making the zone a hard
        blocker on boards with narrow routing channels.
        """
        indexed = self.zone_fill_indexes.get(int(layer))
        if indexed is None:
            return 0.0
        tree, entries = indexed
        segment = LineString([a, b])
        matches = tree.query(segment, predicate="intersects")
        if not len(matches):
            return 0.0
        crossing_count = 0
        inside_length = 0.0
        for raw_index in matches:
            zone_net, polygon = entries[int(raw_index)]
            if zone_net == net:
                continue
            length = segment.intersection(polygon).length
            if length > EPS:
                crossing_count += 1
                inside_length += length
        if not crossing_count:
            return 0.0
        base = max(0.5, 2.0 * (self.clearance + self.trace_w))
        return crossing_count * base + inside_length

    def use_net_rule(self, net):
        """Select this net's width/clearance/via rules before route search."""
        rule = rule_for_net(self.drc, net)
        self.clearance = float(rule["clearance"])
        self.trace_w = float(rule["trace_width"])
        self.via_drill = float(rule["via_drill"])
        self.via_size = max(float(rule.get("via_track_width", 0.6)),
                            self.via_drill + 2.0 * float(rule.get("via_annular_ring", 0.15)))
        self.via_r = self.via_size / 2.0
        self.obst.set_rule(self.clearance, self.trace_w)
        x1, y1, x2, y2 = self.board_box.bounds
        oc = float(rule.get("outline_clearance", self.drc.get("outline_clearance", 0.5)))
        w = self.trace_w / 2.0 + SAFETY
        self.keep_wire = box(x1 + oc + w, y1 + oc + w, x2 - oc - w, y2 - oc - w)
        r = self.via_r + SAFETY
        self.keep_via = box(x1 + oc + r, y1 + oc + r, x2 - oc - r, y2 - oc - r)

    def emit_track(self, net, layer, a, b):
        self.items.append({"type": "track", "net": net, "layer": int(layer),
                           "width": self.trace_w,
                           "a": [round(a[0], 4), round(a[1], 4)],
                           "b": [round(b[0], 4), round(b[1], 4)]})
        self.obst.add(layer, LineString([a, b]).buffer(self.trace_w / 2.0), net)

    def hole_ok(self, p):
        """True if a via drill at p keeps hole_clearance from every drilled
        hole on the board (pads + already-emitted vias, any net)."""
        r = self.via_drill / 2.0
        for h in self._holes:
            if math.hypot(h[0] - p[0], h[1] - p[1]) < h[2] + r + self.hole_clear + 0.05:
                return False
        return True

    def emit_via(self, net, p):
        # never stack a via on an existing hole: a through pad's own drill is
        # already there (holes_co_located), and a duplicate via of this same
        # net at the same spot is invisible to the obstacle model.
        key = (round(p[0], 3), round(p[1], 3))
        if key in self._via_pts or not self.hole_ok(p):
            return
        self._via_pts.add(key)
        self.items.append({"type": "via", "net": net,
                           "size": self.via_size, "drill": self.via_drill,
                           "p": [round(p[0], 4), round(p[1], 4)]})
        self._holes.append((p[0], p[1], self.via_drill / 2.0, net))
        disc = Point(p).buffer(self.via_r)
        for layer in self.copper:
            self.obst.add(layer, disc, net)

    def snapshot(self):
        """Cheap full copy of the routable state (shapely polys are
        immutable, so the lists are the only thing that mutates)."""
        return (list(self.items), list(self._holes), set(self._via_pts),
                {l: list(v) for l, v in self.obst.entries.items()})

    def restore(self, snap):
        self.items = list(snap[0])
        self._holes = list(snap[1])
        self._via_pts = set(snap[2])
        self.obst.entries = {l: list(v) for l, v in snap[3].items()}
        self.obst._trees = {}          # lazy rebuild with current net filter
        self.obst._pts_cache = {}
        self.obst._wgrid_cache = {}

    def rip_net(self, net):
        """Withdraw one net's ROUTES (tracks, vias) from the item list, the
        obstacle model and the hole guards. Pads stay: they are board facts,
        not routing decisions. Used by rip-up-and-reroute."""
        self.items = [it for it in self.items if it["net"] != net]
        for layer in list(self.obst.entries):
            ent = self.obst.entries[layer]
            kept = [e for e in ent if e[3] != net]
            if len(kept) != len(ent):
                if kept:
                    self.obst.entries[layer] = kept
                else:
                    del self.obst.entries[layer]
                self.obst._trees.pop(layer, None)
                self.obst._pts_cache.pop(layer, None)
                self.obst._wgrid_cache.pop(layer, None)
        # Obstacle entries use the electrical net as their exclusion tag for
        # both pads and emitted routes. The filter above therefore removes
        # the victim's permanent pads together with its mutable copper. Put
        # every pad back before another net is searched; otherwise a target
        # transaction can legally pass through the temporarily ripped net's
        # footprint and commit a physical short.
        for pad in self.pads:
            if pad.net != net:
                continue
            for layer in pad.accessible(self.copper):
                self.obst.add(layer, pad.rect(), pad.net,
                              pad.local_clearance)
        self._holes = [h for h in self._holes if h[3] != net]
        # rebuild the via dedup set from what remains: emit_via refuses a
        # coordinate already in _via_pts, so stale entries would silently
        # swallow the re-route's vias. Pad holes are tagged None and stay.
        self._via_pts = {(round(it["p"][0], 3), round(it["p"][1], 3))
                         for it in self.items if it.get("type") == "via"}
        self.obst._trees = {}

    def escape_points(self, pad):
        out = []
        base = pad.rmax + self.obst.margin + 0.05
        angs = set(range(0, 360, 45))
        rot_deg = math.degrees(pad.rot) % 360.0
        if 0.5 < rot_deg < 359.5:
            # a rotated pad's own axis is the natural fan-out direction and
            # the only one that clears fine pitch — it is generally NOT among
            # the eight 45-degree probes.
            angs.update((int(round(rot_deg)), int(round((rot_deg + 180.0) % 360.0))))
        for ang in sorted(angs):
            a = math.radians(ang)
            out.append((pad.x + base * math.cos(a), pad.y + base * math.sin(a)))
        # A pad close to an edge can have its normal outward escape point
        # beyond the legal trace-centre box. Add a point just inside that
        # boundary so the search can leave the pad outward and turn back
        # around adjacent fine-pitch pins. The extra 3*SAFETY accounts for
        # the exact Edge.Cuts obstacle inflation being slightly stricter than
        # the rectangular keep-in approximation.
        x1, y1, x2, y2 = self.keep_wire.bounds
        inset = 3.0 * SAFETY
        if pad.x + base > x2 and pad.x < x2 - inset:
            out.append((x2 - inset, pad.y))
        if pad.x - base < x1 and pad.x > x1 + inset:
            out.append((x1 + inset, pad.y))
        if pad.y + base > y2 and pad.y < y2 - inset:
            out.append((pad.x, y2 - inset))
        if pad.y - base < y1 and pad.y > y1 + inset:
            out.append((pad.x, y1 + inset))
        return out
