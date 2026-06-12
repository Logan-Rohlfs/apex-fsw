#!/usr/bin/env python3
"""Deprecated compatibility shim — the ground app is now HORIZON.

The monitor was renamed: use `python scripts/horizon.py`. This module
re-exports everything from `horizon` so existing imports and the old
`python scripts/monitor.py` habit keep working, but new code should import
and run `horizon` directly.
"""

from horizon import *      # noqa: F401,F403 — re-export the whole app
from horizon import main   # noqa: F401

if __name__ == "__main__":
    main()
