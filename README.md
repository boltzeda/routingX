# Standalone PCB router

This accepts a placed KiCad board and routes a separate output
board. It does not create a board, place or move components, or select another
router.

## Python file guide

The user-facing entry point is `router_tool/__main__.py`. The implementation
is split by routing responsibility under `router_tool/engine/`.

### Package and command files

- `router_tool/__init__.py` identifies the Python package and stores its
  version. It does not start routing.
- `router_tool/__main__.py` makes `python -m router_tool ...` work. It calls
  `cli.main()` and exits with that function's status code.
- `router_tool/cli.py` owns the public command interface:
  - `_parser()` defines the board, output, DRC, via-cost, and quiet options.
  - `main(argv)` parses those options, calls `route_board()`, prints the JSON
    result, and returns `0` for complete, `2` for partial, or `1` for error.
  - `route_board(input_board, output_board, ...)` validates paths and inputs,
    calls the routing pipeline, and writes the user-facing routing-result
    JSON. It refuses to overwrite the source board.
  - `_load_drc(path)` reads an optional routing-rules JSON object.
  - `_load_lineexplore()` loads the algorithm implementation from this
    directory's `engine/` folder.
  - `_result_sidecar(output_board)` determines the result JSON filename.

### Shared settings and rule validation

- `engine/router_constants.py` stores shared constants: nanometres per
  millimetre, default DRC values, geometric safety margins, and search,
  recovery, and time budgets. There are no functions in this file.
- `engine/routing_rules.py` validates and resolves board-wide and per-net
  routing rules:
  - `_positive(value, label)` requires a finite positive numeric rule value.
  - `normalize_routing_rules(drc, known_nets)` validates rule fields and net
    classes, including duplicate assignments and optionally unknown nets.
  - `rules_by_net(drc)` expands each class into a complete set of rules for
    every net in that class.
  - `rule_for_net(drc, net)` returns the board defaults with that net's class
    overrides applied.
  - `minimum_board_rules(drc)` returns the smallest configured value for each
    rule across the board and all its classes.

### Reading the board and representing copper

- `engine/board_geometry.py` reads the existing PCB into a geometry dictionary;
  it does not place or move footprints:
  - `_board_routing_floors(board)` reads the board's minimum track, clearance,
    via, and hole dimensions.
  - `_clamp_routing_rules(drc, floors)` raises supplied defaults and net-class
    rules to the KiCad minima and ensures the via diameter accommodates its
    drill and annular ring.
  - `_clamp_routing_rules_for_board(board_file, drc)` loads the board and
    applies that clamp.
  - `extract_geometry(board_file, drc, verbose)` collects copper layers,
    pads, board bounds, edge segments, copper graphics, filled zones, zone
    thermal settings, and rules into JSON-serializable data. Its nested
    `add_copper_graphic(...)` helper converts unnetted copper artwork into
    polygon obstacles.
- `engine/search_model.py` contains the objects the geometric search operates
  on:
  - `Obstacles` stores per-layer copper geometry. `add()` creates clearance
    buffers; `set_rule()` rebuilds them for a different net class;
    `rebuild_for_net()` selects the net being routed; `_ensure()` builds
    spatial indexes; `layer_points()` supplies obstacle waypoints;
    `_wire_grid()` builds a fast bounding-box grid; `wire_clear()` checks a
    trace segment; and `via_clear()` checks a via across copper layers.
  - `Pad` stores a pad's location, shape, net, and accessible layers.
    `rect()` returns its copper shape, `_fallback_rect()` builds an oriented
    rectangle when exact geometry is unavailable, and `accessible()` returns
    the layers that pad can connect to.
  - `Env` owns the current routing state: pads, obstacles, zones, active net
    rules, emitted routes, holes, and statistics. `inside_filled_zone()` checks
    zone membership; `zone_crossing_cost()` assigns a cost for crossing a
    foreign pour; `use_net_rule()` selects a net's rules; `emit_track()` and
    `emit_via()` add routes to the working state; `hole_ok()` checks drill
    spacing; `snapshot()` and `restore()` support transactional recovery;
    `rip_net()` removes a net's route while retaining its pad obstacles; and
    `escape_points()` proposes legal exits around a pad.

### Searching, planning, and shaping routes

- `engine/path_search.py` connects pads using the sparse waypoint graph:
  - `_route_pair(...)` searches between two pads using direct-path checks and
    A*-style waypoint search, including layer changes when a via is allowed.
    Its local helpers track search statistics, trim legal pad fan-outs for
    collision checking, select nearby corners, check board containment and
    via legality, memoize collision and zone-cost checks, and calculate search
    costs.
  - `_route_pair_adaptive(...)` tries a limited local search before allowing
    the wider search.
  - `_emit_path(...)` turns a found path into track and via entries in `Env`.
  - `_dist_to_segs(...)` finds the nearest distance from a pad to the current
    route tree; `_path_segments(...)` lists the tree's segments.
  - `_route_pair_process(...)` is the worker target for one path-search job;
    `_route_pair_parallel(...)` runs candidate anchor searches concurrently.
  - `_route_chained_net(...)` connects a multi-pad net as a tree, repeatedly
    joining an unconnected pad to the already connected pads or traces.
- `engine/route_planner.py` decides route order and recovery:
  - `_span(multi, net)` estimates a net's physical spread for ordering.
  - `_stitch_pad(...)` tries to connect a surface-mount pad to its copper zone
    with a short trace and via.
  - `_riproute(...)` uses observed blockers to try bounded rip-up-and-reroute
    transactions. It temporarily removes blocker nets, attempts repairs, and
    keeps a transaction only when the board-wide open-connection score
    improves. Its local `record(...)` helper saves diagnostics, while
    `connectivity_score(...)` compares trial outcomes.
  - `_pairs(seq)` generates unique pairs of elements.
  - `_attempt(...)` runs one routing attempt: initialize obstacles, route
    ordinary nets, recover blocked nets, then stitch zone nets. It returns the
    emitted items, failures, and attempt statistics.
  - `route_geometry(geo, ...)` is the geometry-only coordinator. It builds
    nets from pads, chooses route order, runs the main attempt and limited
    alternate orders, and retains the result with the fewest open connections.
    Its local `run(...)` helper executes and scores one attempt; the best
    attempt can be checkpointed in the routes JSON during the run.
- `engine/route_refinement.py` converts track segments to legal octilinear
  geometry while preserving endpoints, connectivity, and vias:
  - `_is_135_segment(...)` identifies already-horizontal, vertical, or
    45-degree segments.
  - `_parallelogram_candidates(...)` makes the two candidate 135-degree paths
    for an arbitrary-angle segment; `_merge_135_points(...)` removes duplicate
    and redundant points.
  - `_post_add_cached(...)` adds a cached obstacle shape;
    `_postprocess_obstacles(...)` builds the obstacles for the net currently
    being refined.
  - `_post_thermal_pads(...)` and `_post_segment_thermal_limits(...)` protect
    thermal relief spacing around zone-connected pads.
  - `_post_path_clear(...)` tests a candidate path; `_post_intersection_parameters(...)`
    finds obstacle intersection points that can be used to split a blocked
    segment; `_refine_135_segment(...)` recursively tries safe replacement
    geometry.
  - `postprocess_135_routes(...)` applies this to routed tracks and records
    conversions, failures, and before/after wirelength. If a replacement fails,
    it keeps that track's original geometry.

### Applying and checking the board

- `engine/board_apply.py` writes routes into a separate board file:
  - `_strip_tracks_text(board_file)` makes a temporary board copy without
    existing track, via, and arc records, leaving the source untouched.
  - `apply_routes(board_file, out_file, routes, drc)` adds route tracks and
    vias using the selected net rules, refills zones, and saves the output PCB.
- `engine/board_audit.py` checks zone connectivity and native DRC:
  - `_audit_zone_pads_on(board, geo)` uses KiCad connectivity data to find
    zone-net pad groups the refill did not connect. It selects connected
    anchors and unconnected representatives for escape routing. Its local
    `component_groups(...)` groups pads belonging to the same connected
    component; `add(...)` avoids duplicate escape records.
  - `audit_zone_pads(board_file, geo)` loads a board and performs that check;
    `_audit_subprocess(...)` runs it in a fresh worker process.
  - `_native_kicad_drc(board_file)` runs `kicad-cli pcb drc` and returns the
    JSON report.
  - `_violation_identity(...)` creates a stable comparison key for a DRC
    finding; `_new_drc_findings(before, after, ...)` returns findings added by
    a candidate route or refinement.
  - `_project_for_board(source_board)` locates the project file used for its
    rules.
  - `_audit_final_connectivity(board_file, source_board, report_override)`
    compares the routed board with a track-stripped baseline and reports open
    connections, total violations, and violations introduced by routing. Its
    local `run_drc(...)` runs DRC; `violation_key(...)` supports the comparison.

### Coordinating the phases

- `engine/pipeline.py` coordinates geometry, search, refinement, board writing,
  and audits:
  - `_geometry_stage(...)` reads geometry JSON, runs `route_geometry(...)`,
    and writes route JSON. `_run_geometry_subprocess(...)` starts it in a
    separate process with a deadline and can read a saved checkpoint if the
    worker exits unexpectedly.
  - `_postprocess_stage(...)` loads route and geometry data, refines the
    routes, and atomically saves the result. `_run_postprocess_subprocess(...)`
    runs that work separately and preserves raw routes if shaping fails.
  - `_load_drc(...)` loads supplied rules or defaults.
    `_apply_subprocess(...)` runs `apply_routes(...)` in a separate process.
  - `route_board_file(...)` is the main library interface. It clamps rules,
    extracts geometry, searches, applies routes, attempts zone escapes, runs
    route shaping, and collects the final connectivity and DRC statistics. It
    returns `(ok, stats)`.
  - `run_lineexplore(...)` adapts the library interface to a directory and
    filename prefix. It backs up the board, routes it in place for callers
    using that interface, and writes the routing-result sidecar.
  - `main()` handles the internal worker-stage command options. It is called
    through the compatibility file below, not by the user-facing package CLI.
- `engine/worker.py` is the small compatibility/worker entry point. It
  re-exports `route_board_file(...)` and `run_lineexplore(...)`, then delegates
  internal subprocess commands to `pipeline.main()`. Keeping this file lets
  the worker processes use one stable script path while the implementation is
  split across modules.

## Entry point

From the repository root:

```bash
python -m router_tool path/to/placed_board.kicad_pcb
```

The default output is `placed_board_lineexplore_routed.kicad_pcb` next to the
input. The source file is preserved. A JSON routing result is written beside
the output as `<output-stem>_routing_result.json`; LineExplore also writes its
route details sidecar.

Optional arguments:

```text
--out <path>       choose the routed board output path
--drc <path.json>  provide routing-rule values
--via-cost <value> set the router's via cost (default 2.0)
--quiet            reduce routing progress output
```

Exit codes are `0` for complete routing, `2` for a partial route, and `1` for
an input or execution error. A partial board is still emitted for inspection;
the result records it as partial and does not mark it manufacturing-ready.
Run the board's native DRC separately before treating a result as ready for
manufacturing.

## Runtime requirements

The pip dependencies are listed in `router_tool/requirements.txt`.

The router also requires a system KiCad installation that provides both the
`pcbnew` Python binding and the `kicad-cli` executable. These are system
dependencies and cannot be installed from `requirements.txt`. The project's
runtime is Python 3.12, matching KiCad 10's `pcbnew` binding on Ubuntu 24.04.
Use a Python interpreter compatible with the installed KiCad binding; do not
use Python 3.13 with this KiCad 10 binding. Create the environment with system
packages visible:

```bash
python3 -m venv --system-site-packages .venv-router
.venv-router/bin/python -m pip install -r router_tool/requirements.txt
```

The router does not import the placement pipeline or the plugin's layout
runner.
