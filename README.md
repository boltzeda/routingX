# BoltzPCB Router

BoltzPCB Router routes an already placed KiCad PCB and writes a separate
output board. It does not create a board or place or move components. The
input board is preserved.

## Requirements

- A Python interpreter compatible with the installed KiCad `pcbnew` binding
- A KiCad installation with the `pcbnew` Python binding and `kicad-cli`
- Python packages listed in `requirements.txt`

The router searches common KiCad installation locations on Windows, macOS,
and Linux. The `pcbnew` binding and `kicad-cli` come from KiCad and are not
installed by pip. The Python interpreter must be compatible with the installed
KiCad binding.

## Setup

From the directory containing the `Boltzpcb_router/` folder:

```bash
python -m venv --system-site-packages Boltzpcb_router/.venv-router
```

Activate the environment with PowerShell on Windows:

```powershell
.\Boltzpcb_router\.venv-router\Scripts\Activate.ps1
```

On macOS or Linux:

```bash
source Boltzpcb_router/.venv-router/bin/activate
```

Then install the dependencies:

```bash
python -m pip install -r Boltzpcb_router/requirements.txt
```

If automatic discovery does not find KiCad, set `KICAD_ROOT` to its install
root. You can set `KICAD_PYTHON_PATH` to the directory containing `pcbnew.py`
or `_pcbnew`, `KICAD_BIN` to the directory containing KiCad's native
libraries, or `KICAD_CLI` to the `kicad-cli` executable or its directory.

This project currently runs from a source checkout; it is not published as an
installable Python package.

## Usage

```bash
python -m Boltzpcb_router path/to/placed_board.kicad_pcb
```

By default, a complete route is written as `<board>_routed.kicad_pcb` beside
the input. A partial route is written as `<board>_partial.kicad_pcb`. With
`--out`, complete routes use the requested path and partial routes add
`_partial` before the file extension:

```bash
python -m Boltzpcb_router path/to/placed_board.kicad_pcb \
  --out path/to/routed_board.kicad_pcb
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
