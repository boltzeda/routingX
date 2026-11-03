"""Shared geometry units and routing budgets for the LineExplore engine."""

NM = 1_000_000          # nm per mm
EPS = 1e-4              # 0.1 um geometric slack so "exactly at clearance" passes
SAFETY = 0.02           # 20 um router safety over nominal clearance
MAX_CORNERS = 96        # nearest waypoints considered per A* expansion
MAX_EXPANSIONS = 12000  # per pair; hopeless searches fail fast (attempts retry)
NET_TIME_BUDGET = 45.0  # seconds across all pad connections for one net
BOARD_TIME_BUDGET = 540.0  # routing/escape-search budget
ZONE_ESCAPE_BUDGET = 180.0  # reserve time for zone-refill connectivity repair
PRIMARY_ROUTE_BUDGET = 360.0
RIPUP_TOTAL_BUDGET = 90.0
RIPUP_BUDGET = RIPUP_TOTAL_BUDGET
INITIAL_RIPUP_BUDGET = RIPUP_TOTAL_BUDGET
ALTERNATE_ORDER_BUDGET = 90.0
RIPUP_PER_NET_BUDGET = 8.0
MIN_RIPUP_PER_NET_BUDGET = 1.0
MAX_RIPUP_COMBOS_PER_NET = 2
MAX_ALTERNATE_ORDERS = 2
MAX_PARALLEL_PATHS = 6
ADAPTIVE_WINDOWS = 1
ADAPTIVE_EXPANSIONS = 1500
ADAPTIVE_MIN_PADS = 64
GRID_CELL = 2.0
GC = 4.0
WCELL = 1.5
GAP_PROBE = 1.5

DEFAULT_DRC = {
    "trace_width": 0.25, "clearance": 0.2, "via_drill": 0.3,
    "via_annular_ring": 0.15, "via_track_width": 0.6,
    "outline_clearance": 0.5, "hole_clearance": 0.25,
}

__all__ = [
    "NM", "EPS", "SAFETY", "MAX_CORNERS", "MAX_EXPANSIONS",
    "NET_TIME_BUDGET", "BOARD_TIME_BUDGET", "ZONE_ESCAPE_BUDGET",
    "PRIMARY_ROUTE_BUDGET", "RIPUP_TOTAL_BUDGET", "RIPUP_BUDGET",
    "INITIAL_RIPUP_BUDGET",
    "ALTERNATE_ORDER_BUDGET", "RIPUP_PER_NET_BUDGET",
    "MIN_RIPUP_PER_NET_BUDGET", "MAX_RIPUP_COMBOS_PER_NET",
    "MAX_ALTERNATE_ORDERS", "MAX_PARALLEL_PATHS", "ADAPTIVE_WINDOWS",
    "ADAPTIVE_EXPANSIONS", "ADAPTIVE_MIN_PADS", "GRID_CELL", "GC",
    "WCELL", "GAP_PROBE", "DEFAULT_DRC",
]
