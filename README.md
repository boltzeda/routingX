# BoltzPCB Router

BoltzPCB Router routes an already placed KiCad PCB.

## Prerequisites

### KiCad 10

Install KiCad 10 with the PCB Editor's `pcbnew` Python binding and `kicad-cli`. If KiCad is not installed, download the installer for your operating system from the [official KiCad download page](https://www.kicad.org/download/). Linux users can also follow KiCad's [distribution-specific installation instructions](https://www.kicad.org/download/linux-distros/).

```bash
kicad-cli version
```

### Python version

Python **>= 3.13** is required to run the router

### Python dependencies

Install the pinned Python packages from `requirements.txt`

```bash
python -m pip install -r Boltzpcb_router/requirements.txt
```
## Usage

Run the command from the directory containing the `Boltzpcb_router/` package folder:

```bash
python -m Boltzpcb_router path/to/placed_board.kicad_pcb
```

By default, a complete route is written as `<board>_routed.kicad_pcb` beside the input. A partial route is written as `<board>_partial.kicad_pcb`.

```bash
python -m Boltzpcb_router path/to/placed_board.kicad_pcb \
  --out path/to/routed_board.kicad_pcb
```

Options:

- `--out <path>`: choose the routed board output path.
- `--drc <path.json>`: optionally provide routing-rule values. If omitted, the router uses its built-in defaults and raises them to the board's minimum constraints.
- `--via-cost <value>`: set the via cost; defaults to `2.0`.
- `--quiet`: reduce progress output.

The router writes a routing-result JSON file beside the output and a route details file at `<output-path>.routes.json`. A partial board is emitted for inspection when some connections remain unrouted. Exit codes are `0` for complete routing, `2` for a partial route, and `1` for an input or execution error.

Always run KiCad's native DRC on the output before relying on it. A completed route is not a manufacturing-readiness certification.

## License

This project is released under the MIT License. See [LICENSE](LICENSE).
