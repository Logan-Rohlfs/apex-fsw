"""Filesystem locations for the log archive pipeline, plus the typed
confirmation phrases for destructive operations.

Flight Computer (device files) → Laptop Archive (raw .APXLOG binaries) → CSV
Exports (decoded per-flight CSVs).
"""

from __future__ import annotations

from pathlib import Path

# scripts/horizon_app/paths.py -> parents[2] == sim/
_SIM_ROOT = Path(__file__).resolve().parents[2]
_RAW_LOG_ARCHIVE = _SIM_ROOT / "output" / "raw_logs"
_FC_LOG_ARCHIVE = _RAW_LOG_ARCHIVE / "flight_computer"
_DELETED_LOG_ARCHIVE = _SIM_ROOT / "output" / "raw_logs_deleted"
_DELETE_CONFIRM_PHRASE = "yes i really do want to delete these files"
_FORMAT_QSPI_CONFIRM_PHRASE = "yes i really do want to format qspi flash"
