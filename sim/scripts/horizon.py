#!/usr/bin/env python3
"""
HORIZON — Apex Ground (launcher).

The implementation lives in the horizon_app/ package (split out of what used to
be one ~4300-line module): theme, workers, widgets, state panel, the adaptive
connection bar, the Logs pipeline, and the main window. This file stays as the
entry point so the run command is unchanged:

    python scripts/horizon.py

`import horizon` still exposes MainWindow / main for the test suite.
(scripts/monitor.py is a deprecated compatibility shim.)
"""

import sys
from pathlib import Path

# scripts/ (radio_gfsk_rx, horizon_app) and sim/ (apex_sim) on the path so the
# package imports resolve whether launched as a script or imported as `horizon`.
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from horizon_app import MainWindow, main   # noqa: E402,F401

if __name__ == "__main__":
    main()
