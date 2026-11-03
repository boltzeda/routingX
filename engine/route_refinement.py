"""Clearance-aware 135-degree route geometry refinement."""

import math

from shapely.geometry import LineString, Point, box, Polygon
from shapely import affinity
from shapely.strtree import STRtree

from router_constants import *  # noqa: F403
from search_model import Env, Obstacles


def _is_135_segment(a, b, tol=1e-7):
    """True for an axis-aligned or 45-degree segment.

    Consecutive segments produced by the parallelogram construction use only
    the major axis and the matching diagonal direction, so every non-straight
    turn has a 135-degree interior angle.
    """
    dx = abs(b[0] - a[0])
    dy = abs(b[1] - a[1])
    return dx <= tol or dy <= tol or abs(dx - dy) <= tol


def _parallelogram_candidates(a, b):
    """Return the two 135-degree paths forming the segment parallelogram.

    For a non-octilinear chord, one side travels along the dominant axis and
    then diagonally; the opposite side travels diagonally and then along the
    dominant axis. Together the two alternatives are the parallelogram used
    by the paper's wire-shape optimization stage.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    ax, ay = abs(dx), abs(dy)
    if _is_135_segment(a, b):
        return [[a, b]]
    sx = 1.0 if dx >= 0.0 else -1.0
    sy = 1.0 if dy >= 0.0 else -1.0
    if ax > ay:
        # horizontal -> diagonal, or diagonal -> horizontal
        p_axis = (a[0] + sx * (ax - ay), a[1])
        p_diag = (a[0] + sx * ay, a[1] + sy * ay)
    else:
        # vertical -> diagonal, or diagonal -> vertical
        p_axis = (a[0], a[1] + sy * (ay - ax))
        p_diag = (a[0] + sx * ax, a[1] + sy * ax)
    return [[a, p_axis, b], [a, p_diag, b]]


def _merge_135_points(points):
    """Drop duplicate and forward-collinear points from an octilinear path."""
    out = []
    for p in points:
        p = (round(float(p[0]), 6), round(float(p[1]), 6))
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) <= 1e-7:
            continue
        out.append(p)
        while len(out) >= 3:
            a, m, b = out[-3], out[-2], out[-1]
            u = (m[0] - a[0], m[1] - a[1])
            v = (b[0] - m[0], b[1] - m[1])
            cross = u[0] * v[1] - u[1] * v[0]
            if abs(cross) > 1e-8 or u[0] * v[0] + u[1] * v[1] < 0.0:
                break
            out[-2:] = [b]
    return out


def _post_add_cached(obst, layer, raw_factory, net, key, cache,
                     local_clearance=0.0):
    """Add an obstacle entry while reusing its expensive Shapely buffers."""
    local_clearance = max(0.0, float(local_clearance))
    key = (round(obst.clearance, 6), round(obst.trace_w, 6),
           round(local_clearance, 6), key)
    entry = cache.get(key)
    if entry is None:
        raw = raw_factory()
        extra = max(0.0, local_clearance - obst.clearance)
        wire = raw.buffer(obst.margin + extra, join_style=2)
        try:
            nvert = len(raw.exterior.coords)
        except AttributeError:
            nvert = 99
        if nvert <= 8:
            corner = raw.buffer(obst.corner_margin + extra, join_style=2)
        else:
            corner = box(*raw.bounds).buffer(
                obst.corner_margin + extra, join_style=2)
        entry = (raw, wire, corner, net, local_clearance)
        cache[key] = entry
    obst.entries.setdefault(layer, []).append(entry)


def _postprocess_obstacles(env, geo, items, current_net, cache):
    """Build the obstacle model used only by 135-degree post-processing.

    `items` contains already-normalized earlier nets and original later nets.
    Rebuilding per net makes every replacement see the latest geometry while
    excluding its own copper, which preserves multi-pin junctions.
    """
    env.use_net_rule(current_net)
    obst = Obstacles(env.clearance, env.trace_w)
    for pad_index, pad in enumerate(env.pads):
        for layer in pad.accessible(env.copper):
            _post_add_cached(obst, layer, pad.rect, pad.net,
                             ("pad", pad_index, layer), cache,
                             pad.local_clearance)

    for item_index, item in enumerate(geo.get("copper_graphics") or []):
        layer = int(item["layer"])
        raw_poly = item["poly"]
        holes = item.get("holes") or []

        def graphic_raw(raw_poly=raw_poly, holes=holes):
            return Polygon(raw_poly, holes)

        _post_add_cached(
            obst, layer, graphic_raw, None,
            ("copper_graphic", item_index, layer), cache)

    oc = float((geo.get("drc") or {}).get("outline_clearance",
                                           DEFAULT_DRC["outline_clearance"]))
    for edge_index, ex in enumerate(geo.get("edges") or []):
        extra = max(0.0, oc - env.clearance)
        for layer in env.copper:
            def edge_raw(ex=ex, extra=extra):
                return LineString([(ex[0], ex[1]), (ex[2], ex[3])]) \
                    .buffer(extra + 0.001, cap_style=2)
            _post_add_cached(obst, layer, edge_raw, None,
                             ("edge", edge_index, layer, round(extra, 6)), cache)

    for item in items:
        if item.get("type") == "track" and item.get("a") != item.get("b"):
            layer = int(item["layer"])
            a, b = tuple(item["a"]), tuple(item["b"])
            item_width = float(item.get("width", env.trace_w))
            def track_raw(a=a, b=b, item_width=item_width):
                return LineString([a, b]).buffer(item_width / 2.0, cap_style=2)
            _post_add_cached(obst, layer, track_raw, item["net"],
                             ("track", layer, item["net"], a, b), cache)
        elif item.get("type") == "via":
            p = tuple(item["p"])
            item_radius = float(item.get("size", env.via_size)) / 2.0
            for layer in env.copper:
                def via_raw(p=p, item_radius=item_radius):
                    return Point(p).buffer(item_radius)
                _post_add_cached(obst, layer, via_raw, item["net"],
                                 ("via", layer, item["net"], p), cache)
    obst.rebuild_for_net(current_net)
    return obst


def _post_thermal_pads(env, geo):
    """Return thermal-pad guard records for post-processing only."""
    gaps = {}
    records = geo.get("zone_thermals") or []
    if records:
        for rec in records:
            if int(rec.get("connection", 1)) not in (1, 3):
                continue
            key = (rec.get("net"), int(rec.get("layer", -1)))
            gaps[key] = max(gaps.get(key, 0.0), float(rec.get("gap", 0.5)))
    else:
        for net, layer in (geo.get("zone_map") or {}).items():
            gaps[(net, int(layer))] = 0.5

    guards = []
    for pad in env.pads:
        for layer in pad.accessible(env.copper):
            gap = gaps.get((pad.net, int(layer)))
            if gap is not None:
                # Required centerline-to-pad-copper distance. The source path
                # may already be closer on a DRC-clean board, so each segment
                # later receives a baseline-preserving effective limit.
                required = gap + env.trace_w / 2.0 + SAFETY
                guards.append((int(layer), pad.net, pad.rect(), required))
    return guards


def _post_segment_thermal_limits(thermal_pads, current_net, layer, a, b):
    """Do not let normalization consume more spoke room than the source."""
    source = LineString([a, b])
    limits = []
    for guard_layer, pad_net, pad_poly, required in thermal_pads:
        if guard_layer != layer or pad_net == current_net:
            continue
        baseline = source.distance(pad_poly)
        limits.append((pad_poly, min(required, baseline)))
    return limits


def _post_path_clear(env, obst, layer, points, thermal_limits=()):
    # Recursive refinement can produce subsegments shorter than the six-place
    # coordinate precision used by _merge_135_points. Both endpoints then
    # collapse to one distinct point, which is not a valid Shapely LineString.
    points = _merge_135_points(points)
    if len(points) < 2:
        return False
    kx1, ky1, kx2, ky2 = env.keep_wire.bounds
    for a, b in zip(points, points[1:]):
        if (min(a[0], b[0]) < kx1 or max(a[0], b[0]) > kx2
                or min(a[1], b[1]) < ky1 or max(a[1], b[1]) > ky2):
            return False
        if not obst.wire_clear(layer, a, b):
            return False
    if thermal_limits:
        path = LineString(points)
        for pad_poly, limit in thermal_limits:
            if path.distance(pad_poly) + EPS < limit:
                return False
    return True


def _post_intersection_parameters(obst, layer, candidates, a, b):
    """Project parallelogram/obstacle intersections onto the source chord."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-18:
        return []
    params = []
    current = obst._current_net
    entries = obst.entries.get(layer, ())
    for points in candidates:
        for p, q in zip(points, points[1:]):
            line = LineString([p, q])
            for _raw, wire, _corner, net, _local_clearance in entries:
                if net == current or not line.intersects(wire):
                    continue
                hit = line.intersection(wire)
                if hit.is_empty:
                    continue
                c = hit.centroid
                t = ((c.x - a[0]) * dx + (c.y - a[1]) * dy) / length2
                if 1e-4 < t < 1.0 - 1e-4:
                    params.append(t)
    return params


def _refine_135_segment(env, obst, layer, a, b, depth=0, budget=None,
                        thermal_limits=()):
    """Construct and, on intersection, iteratively refine a parallelogram.

    The refinement point is derived from the detected obstacle intersections,
    matching the paper's Fig. 6 procedure. Sub-parallelograms converge toward
    the already-clear source chord, without changing its endpoints/topology.
    """
    if budget is None:
        budget = [0]
    budget[0] += 1
    # A malformed/partial input route can already cross foreign copper. No
    # amount of geometric normalization can repair that without rerouting,
    # which this stage must never do. Bound refinement so such a segment is
    # reported and retained instead of expanding an exponential split tree.
    if budget[0] > 1024:
        return None
    if math.hypot(b[0] - a[0], b[1] - a[1]) <= 1e-7:
        return None

    candidates = _parallelogram_candidates(a, b)
    candidates = [_merge_135_points(candidate) for candidate in candidates]
    candidates = [candidate for candidate in candidates
                  if len(candidate) >= 2]
    if not candidates:
        return None
    for candidate in candidates:
        if _post_path_clear(env, obst, layer, candidate, thermal_limits):
            return candidate

    if depth >= 18:
        return None
    params = _post_intersection_parameters(obst, layer, candidates, a, b)
    if params:
        params.sort()
        split = params[len(params) // 2]
        split = max(0.15, min(0.85, split))
    else:
        split = 0.5
    m = (a[0] + split * (b[0] - a[0]),
         a[1] + split * (b[1] - a[1]))
    left = _refine_135_segment(env, obst, layer, a, m, depth + 1, budget,
                               thermal_limits)
    right = _refine_135_segment(env, obst, layer, m, b, depth + 1, budget,
                                thermal_limits)
    if left is None or right is None:
        return None
    return _merge_135_points(left[:-1] + right)


def postprocess_135_routes(geo, result, via_cost=2.0, verbose=True):
    """Apply the paper's optional 135-degree wire-shape optimization stage.

    Track endpoints, net topology, vias and routing failures are untouched.
    Only arbitrary-angle track geometry is replaced. Earlier processed nets
    become obstacles for later nets, while unprocessed nets retain their
    original geometry until their turn.
    """
    env = Env(geo, via_cost=via_cost)
    state = [dict(item) for item in result.get("items", [])]
    net_order = []
    for item in state:
        net = item.get("net")
        if item.get("type") == "track" and net not in net_order:
            net_order.append(net)

    pre_length = sum(math.hypot(i["b"][0] - i["a"][0],
                                i["b"][1] - i["a"][1])
                     for i in state if i.get("type") == "track")
    converted = added = 0
    failed = []
    geometry_cache = {}
    for net in net_order:
        env.use_net_rule(net)
        thermal_pads = _post_thermal_pads(env, geo)
        obst = _postprocess_obstacles(env, geo, state, net, geometry_cache)
        updated = []
        for item in state:
            if item.get("type") != "track" or item.get("net") != net:
                updated.append(item)
                continue
            a, b = tuple(item["a"]), tuple(item["b"])
            if _is_135_segment(a, b):
                updated.append(item)
                continue
            layer = int(item["layer"])
            thermal_limits = _post_segment_thermal_limits(
                thermal_pads, net, layer, a, b)
            try:
                points = _refine_135_segment(
                    env, obst, layer, a, b,
                    thermal_limits=thermal_limits)
            except Exception as exc:
                # Post-processing is a shape-only optimization. A failure in
                # one segment must never discard an otherwise usable (even if
                # partial) routed board or prevent later segments from being
                # normalized. Keep the original segment and record why.
                failed.append(
                    f"{net}:{a}->{b} ({type(exc).__name__}: {exc})")
                updated.append(item)
                continue
            if points is None:
                failed.append(f"{net}:{a}->{b}")
                updated.append(item)
                continue
            converted += 1
            added += max(0, len(points) - 2)
            for p, q in zip(points, points[1:]):
                updated.append({**item,
                                # KiCad stores coordinates at nanometre
                                # precision. Preserve all six mm decimals so
                                # serialization does not turn an exact 45°
                                # segment into a visibly non-octilinear one.
                                "a": [round(p[0], 6), round(p[1], 6)],
                                "b": [round(q[0], 6), round(q[1], 6)]})
        state = updated

    post_length = sum(math.hypot(i["b"][0] - i["a"][0],
                                 i["b"][1] - i["a"][1])
                      for i in state if i.get("type") == "track")
    result["items"] = state
    stats = result.setdefault("stats", {})
    stats.update({"postprocess_135": True,
                  "postprocess_converted": converted,
                  "postprocess_added_segments": added,
                  "postprocess_failures": len(failed),
                  "wirelength_pre_135": round(pre_length, 4),
                  "wirelength_post_135": round(post_length, 4)})
    result.pop("postprocess_failures", None)
    if failed:
        result["postprocess_failures"] = failed
    if verbose:
        print(f"  [LE] 135-degree post-process: {converted} segment(s) "
              f"converted, {len(failed)} failure(s), wirelength "
              f"{pre_length:.2f} -> {post_length:.2f} mm")
    return result
