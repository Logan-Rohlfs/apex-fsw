#!/usr/bin/env python3
"""Decode Apex .APXLOG files into one wide CSV per flight.

Examples:
    python scripts/decode_logs.py /Volumes/APEX-FLASH/APEX --out output/log_exports
    python scripts/decode_logs.py output/raw_logs/*.APXLOG --no-ground
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apex_sim.logs.decoder import decode_files, export_logs, iter_log_paths  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Decode Apex binary flight logs")
    ap.add_argument("inputs", nargs="+",
                    help="APXLOG files or directories containing APXLOG files")
    ap.add_argument("--out", default=None,
                    help="export directory (default: sim/output/log_exports)")
    ap.add_argument("--no-ground", action="store_true",
                    help="only export nonzero flight_id groups")
    ap.add_argument("--summary", action="store_true",
                    help="only print decode summary; do not export CSV")
    args = ap.parse_args()

    paths = list(iter_log_paths(Path(p) for p in args.inputs))
    if not paths:
        print("No .APXLOG files found.", file=sys.stderr)
        return 2

    records, stats = decode_files(paths)
    print(
        f"decoded {stats.records} records from {len(paths)} file(s); "
        f"bad_crc={stats.bad_crc} truncated={stats.truncated} "
        f"resync_bytes={stats.resync_bytes}"
    )
    flights = sorted({r.flight_id for r in records if r.flight_id})
    boots = sorted({r.boot_id for r in records})
    print(f"boot_ids={boots} flight_ids={flights}")

    if args.summary:
        return 0

    out_dir = Path(args.out) if args.out else Path(__file__).resolve().parents[1] / "output" / "log_exports"
    written = export_logs(paths, out_dir, include_ground=not args.no_ground)
    for path in written:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
