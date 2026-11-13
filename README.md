# BoltzPCB Router

BoltzPCB Router routes an already placed KiCad PCB and writes a separate
output board. It does not create a board or place or move components. The
input board is preserved.

## Requirements

- Python 3.12
- KiCad 10, including the `pcbnew` Python binding and `kicad-cli`
- Python packages listed in `requirements.txt`

The documented target is Python 3.12 with KiCad 10 on Ubuntu 24.04. Other
Python and KiCad combinations have not been established. The `pcbnew` binding
and `kicad-cli` come from the system KiCad installation and are not installed
by pip.

## Setup

From the directory containing the `Boltzpcb_router/` folder:

```bash
python3.12 -m venv --system-site-packages Boltzpcb_router/.venv-router
Boltzpcb_router/.venv-router/bin/python -m pip install -r Boltzpcb_router/requirements.txt
```

This project currently runs from a source checkout; it is not published as an
installable Python package.

## Usage

```bash
Boltzpcb_router/.venv-router/bin/python -m Boltzpcb_router path/to/placed_board.kicad_pcb
```

By default, a complete route is written as `<board>_routed.kicad_pcb` beside
the input. A partial route is written as `<board>_partial.kicad_pcb`. With
`--out`, complete routes use the requested path and partial routes add
`_partial` before the file extension:

```bash
Boltzpcb_router/.venv-router/bin/python -m Boltzpcb_router \
  path/to/placed_board.kicad_pcb --out path/to/routed_board.kicad_pcb
```

Options:

- `--out <path>`: choose the routed board output path.
- `--drc <path.json>`: optionally provide routing-rule values. If omitted,
  the router uses its built-in defaults and raises them to the board's minimum
  constraints.
- `--via-cost <value>`: set the via cost; defaults to `2.0`.
- `--quiet`: reduce progress output.

The router writes a routing-result JSON file beside the output and a route
details file at `<output-path>.routes.json`. A partial board is emitted for
inspection when some connections remain unrouted. Exit codes are `0` for
complete routing, `2` for a partial route, and `1` for an input or execution
error.

Always run KiCad's native DRC on the output before relying on it. A completed
route is not a manufacturing-readiness certification.

## License

This project is released under the MIT License. See [LICENSE](LICENSE).
