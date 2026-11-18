# routingX

**routingX** is an automated PCB routing tool designed to route already placed KiCad PCB layouts. It takes a placed PCB as input, identifies unrouted connections, and attempts to complete the routing while respecting the board's design constraints.

The router generates a routed KiCad PCB file along with routing reports that help users inspect the routing results and identify any remaining unrouted connections.

## Features

- **Automated PCB Routing:** Routes unrouted connections in an already placed KiCad PCB.
- **Design Rule Awareness:** Supports routing-rule configuration through an optional DRC JSON file.
- **Configurable Via Cost:** Allows users to control the cost associated with via insertion.
- **Partial Routing Support:** Generates a partially routed PCB when all connections cannot be completed.
- **Routing Reports:** Produces JSON reports containing routing results and route details.
- **KiCad Compatibility:** Uses KiCad's PCB tools and Python bindings for board processing.

## Prerequisites

### 1. KiCad 10

Install KiCad 10 with the PCB Editor's `pcbnew` Python binding and `kicad-cli`.

Download KiCad from the [official KiCad download page](https://www.kicad.org/download/).

Linux users can refer to the [distribution-specific installation instructions](https://www.kicad.org/download/linux-distros/).

Verify the installation:

```bash
kicad-cli version
```

Ensure that the `pcbnew` Python module is accessible from the Python environment used to run the router.

### 2. Python

Python **3.13 or later** is required.

Verify your Python version:

```bash
python --version
```

### 3. Python Dependencies

Install the required Python packages from `requirements.txt`.

```bash
python -m pip install -r routingX/requirements.txt
```

## Installation

Clone the repository:

```bash
git clone <repository-url>
```

Navigate to the repository directory:

```bash
cd <repository-name>
```

Install the dependencies:

```bash
python -m pip install -r routingX/requirements.txt
```

Make sure KiCad 10 and its Python bindings are properly configured before running the router.

## Usage

The router accepts an already placed KiCad PCB file (`.kicad_pcb`) as input.

Run the following command from the directory containing the `routingX/` package folder:

```bash
python -m routingX path/to/placed_board.kicad_pcb
```

By default, the router generates the following output:

- **Complete routing:** `<board>_routed.kicad_pcb`
- **Partial routing:** `<board>_partial.kicad_pcb`

The output files are generated in the same directory as the input board.

### Custom Output Path

To specify a custom output location:

```bash
python -m routingX path/to/placed_board.kicad_pcb \
    --out path/to/routed_board.kicad_pcb
```

### Command-Line Options

| Argument | Description | Default |
|---|---|---|
| Input PCB | Path to the placed KiCad PCB file | Required |
| `--out <path>` | Specifies the output PCB file path | Automatically generated |
| `--drc <path.json>` | Provides custom routing-rule values | Built-in defaults |
| `--via-cost <value>` | Controls the cost associated with via insertion | `2.0` |
| `--quiet` | Reduces progress and informational output | Disabled |

### Example with Custom Parameters

```bash
python -m routingX boards/example.kicad_pcb \
    --out output/example_routed.kicad_pcb \
    --via-cost 2.0 \
    --quiet
```

To provide custom routing rules:

```bash
python -m routingX boards/example.kicad_pcb \
    --drc config/drc_rules.json \
    --out output/example_routed.kicad_pcb
```

If no DRC configuration is provided, the router uses its built-in defaults and raises them to satisfy the board's minimum constraints.

## Output Files

The router generates the following files depending on the routing result:

| Output | Description |
|---|---|
| `*_routed.kicad_pcb` | KiCad PCB file generated when routing is complete. |
| `*_partial.kicad_pcb` | Partially routed PCB file generated when some connections remain unrouted. |
| Routing result JSON | Summary of the routing execution and its result. |
| `<output-path>.routes.json` | Detailed routing information for inspection and analysis. |

The partial PCB output allows users to open the board in KiCad and inspect completed and remaining connections.


## Design Rule Verification

After routing, always perform a Design Rule Check (DRC) using KiCad's native DRC functionality.

A successful routing result indicates that the router completed its routing task. However, it does not guarantee that the board is free from all electrical, physical, or manufacturing-related violations.

Recommended verification:

1. Open the generated PCB file in KiCad PCB Editor.
2. Run the native Design Rule Checker (DRC).
3. Inspect unrouted connections, clearance violations, and other reported issues.
4. Review the board before proceeding to manufacturing.


## License

This project is licensed under the **MIT License**.

See the [LICENSE](LICENSE) file for more information.

## Acknowledgements

routingX is part of the BoltzPCB ecosystem, focused on automating and improving PCB design workflows.