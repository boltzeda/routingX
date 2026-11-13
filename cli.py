"""Command-line entry point for routing an already placed KiCad board."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import traceback
from pathlib import Path
from typing import Any


ENGINE_DIR = Path(__file__).resolve().parent / "engine"
WORKER_SOURCE = ENGINE_DIR / "worker.py"


def _load_lineexplore():
    """Load this directory's router implementation without the layout plugin."""
    if not WORKER_SOURCE.is_file():
        raise FileNotFoundError(
            f"Router worker implementation not found: {WORKER_SOURCE}"
        )
    # The implementation imports its routing-rule module by its local name.
    source_dir = str(ENGINE_DIR)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)

    spec = importlib.util.spec_from_file_location(
        "router_tool_worker_implementation", WORKER_SOURCE
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"could not load router worker from {WORKER_SOURCE}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_drc(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read DRC configuration {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError("DRC configuration must be a JSON object")
    return value


def _result_sidecar(output_board: Path) -> Path:
    return output_board.with_name(f"{output_board.stem}_routing_result.json")


def _partial_output_path(output_board: Path) -> Path:
    stem = output_board.stem
    if stem.endswith("_partial"):
        return output_board
    if stem.endswith("_routed"):
        stem = stem[:-len("_routed")]
    return output_board.with_name(f"{stem}_partial{output_board.suffix}")


def route_board(
    input_board: Path,
    output_board: Path,
    *,
    drc: dict[str, Any] | None = None,
    via_cost: float = 2.0,
    verbose: bool = True,
) -> dict[str, Any]:
    """Route an existing board and return its result record.

    The source board is read as input and never overwritten. Component
    positions and rotations are taken from that board; this tool does not
    place or move footprints.
    """
    source = input_board.expanduser().resolve()
    destination = output_board.expanduser().resolve()
    partial_destination = _partial_output_path(destination)
    if not source.is_file():
        raise FileNotFoundError(f"input board does not exist: {source}")
    if source.suffix.lower() != ".kicad_pcb":
        raise ValueError("input must be a .kicad_pcb board file")
    if source == destination:
        raise ValueError("output board must differ from input; source is preserved")
    if source == partial_destination:
        raise ValueError("partial output board must differ from input; source is preserved")
    if not destination.parent.is_dir():
        destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.parent.is_dir():
        raise NotADirectoryError(destination.parent)
    if not math.isfinite(via_cost) or via_cost < 0:
        raise ValueError("via cost must be a finite, non-negative number")

    lineexplore = _load_lineexplore()
    started = lineexplore.time.perf_counter()
    ok, stats = lineexplore.route_board_file(
        str(source), str(destination), drc or dict(lineexplore.DEFAULT_DRC),
        via_cost=via_cost, verbose=verbose,
    )
    routes_path = Path(str(destination) + ".routes.json")
    if not ok and destination != partial_destination:
        destination.replace(partial_destination)
        partial_routes_path = Path(str(partial_destination) + ".routes.json")
        if routes_path.is_file():
            routes_path.replace(partial_routes_path)
        destination = partial_destination
        routes_path = partial_routes_path
    failures: dict[str, Any] = {}
    if routes_path.is_file():
        try:
            route_data = json.loads(routes_path.read_text(encoding="utf-8"))
            failures = route_data.get("failures") or {}
            stats = route_data.get("stats") or stats
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            pass

    result = {
        "router": "lineexplore",
        "status": "complete" if ok else "partial",
        "input_board": str(source),
        "routed_board": str(destination),
        "routing_result": str(_result_sidecar(destination)),
        "failed_nets": sorted(failures),
        "failed_connections": failures,
        "stats": stats,
        "routing_seconds": round(lineexplore.time.perf_counter() - started, 6),
        "manufacturing_ready": False,
        "requires_drc": True,
    }
    _result_sidecar(destination).write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m Boltzpcb_router",
        description=(
            "Route an already placed KiCad PCB with LineExplore. "
            "The input board is preserved; placement is not performed."
        ),
    )
    parser.add_argument("board", type=Path, help="placed, unrouted .kicad_pcb input")
    parser.add_argument(
        "--out", type=Path,
        help=("board output (default: <input>_routed.kicad_pcb; "
              "partial routes use <input>_partial.kicad_pcb)"),
    )
    parser.add_argument("--drc", type=Path, help="optional routing-rules JSON file")
    parser.add_argument("--via-cost", type=float, default=2.0)
    parser.add_argument("--quiet", action="store_true", help="reduce router progress output")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    source = args.board.expanduser()
    output = args.out or source.with_name(
        f"{source.stem}_routed.kicad_pcb"
    )
    try:
        result = route_board(
            source,
            output,
            drc=_load_drc(args.drc),
            via_cost=args.via_cost,
            verbose=not args.quiet,
        )
    except Exception as exc:
        traceback.print_exc()
        print(json.dumps({"router": "lineexplore", "status": "error", "error": str(exc)}))
        return 1

    print(json.dumps(result))
    return 0 if result["status"] == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
