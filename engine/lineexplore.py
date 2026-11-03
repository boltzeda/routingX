"""Compatibility and subprocess-worker entry point for the router tool."""

import sys
import time

from router_constants import *  # noqa: F403
from pipeline import route_board_file, run_lineexplore, main


if __name__ == "__main__":
    sys.exit(main())
