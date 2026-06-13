#!/usr/bin/env python3
"""Automated HIL self-test — drives a real (or fake) HIL flight and asserts the
flight computer behaves and LOGS like a real flight. This is the "actually
tested, no bandaids" harness: it checks the live TeensyPacket stream and,
with --pull, reflashes the MTP build, pulls the binary flight log off QSPI, and
decodes it to verify the recorded log is flight-equivalent.

Usage
-----
    # Fast in-stream check against whatever HIL build is already flashed:
    sim/.venv/bin/python sim/scripts/hil_selftest.py --port /dev/cu.usbmodemXXXX

    # Software-only (no hardware), validates the runner + fake reference:
    sim/.venv/bin/python sim/scripts/hil_selftest.py --fake

    # Full end-to-end: flash HIL, fly to landing, reflash debug, pull + decode
    # the binary log and assert all phases + deploy + PID + CRC integrity:
    sim/.venv/bin/python sim/scripts/hil_selftest.py --full --pull --flash

Exit code 0 = all assertions passed, 1 = a check failed, 2 = setup error.
"""

from __future__ import annotations

import argparse
import glob
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_SIM_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _SIM_ROOT.parent
sys.path.insert(0, str(_SIM_ROOT))

from apex_sim.config.loader import load_environment            # noqa: E402
from apex_sim.hil.link import HilLink                          # noqa: E402
from apex_sim.hil.protocol import PHASE_NAMES                  # noqa: E402
from apex_sim.hil.runner import run_closed_loop                # noqa: E402
from apex_sim.sim.environment import build_environment         # noqa: E402
from apex_sim.sim.rocket import build_rocket                   # noqa: E402
from apex_sim.logs.decoder import decode_files                 # noqa: E402

import yaml

PIO = str(Path.home() / ".platformio/penv/bin/pio")
_FT_PER_M = 3.28084


class Checks:
    """Collects pass/fail assertions and prints them as it goes."""

    def __init__(self):
        self.failed = 0
        self.passed = 0

    def check(self, name: str, ok: bool, detail: str = ""):
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {name}" + (f" — {detail}" if detail else ""))
        if ok:
            self.passed += 1
        else:
            self.failed += 1


def _pio_upload(env: str, port: str | None) -> None:
    cmd = [PIO, "run", "-e", env, "-t", "upload", "-d", str(_REPO_ROOT / "fsw")]
    if port:
        cmd += ["--upload-port", port]
    print(f"  flashing {env} ...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise RuntimeError(f"upload {env} failed")


def _find_port(wait_s: float = 0.0) -> str | None:
    """Newest usbmodem port; polls up to wait_s for re-enumeration after a flash."""
    deadline = time.monotonic() + wait_s
    while True:
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        if ports:
            return ports[-1]
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.5)


def _load_airbrakes() -> dict:
    return yaml.safe_load(open(_SIM_ROOT / "config" / "airbrakes.yaml"))


# ─── In-stream flight assertions ──────────────────────────────────────────────

def assert_flight(result, full: bool, pad_elev_m: float, checks: Checks) -> None:
    phases = [n for _, n in result.transitions]
    replies = [r.reply for r in result.rows if r.reply is not None]
    max_dep = max((r.deployment_frac for r in replies), default=0.0)
    ticks = len(result.rows)
    miss_rate = result.missed / ticks if ticks else 1.0

    checks.check("reached BOOST", "BOOST" in phases, str(phases))
    checks.check("reached COAST", "COAST" in phases)
    if full:
        checks.check("reached DESCENT", "DESCENT" in phases)
        checks.check("reached LANDED (real touchdown + stillness)",
                     "LANDED" in phases,
                     "no LANDED — descent may have been truncated"
                     if "LANDED" not in phases else "")
    checks.check("brakes deployed", max_dep > 0.3, f"max deploy {max_dep*100:.0f}%")
    checks.check("no CRC errors on the link", result.crc_errors == 0,
                 f"{result.crc_errors} CRC errors")
    checks.check("packet miss rate < 2%", miss_rate < 0.02,
                 f"{miss_rate*100:.1f}% ({result.missed}/{ticks})")
    apo = (result.flight.apogee - pad_elev_m) * _FT_PER_M
    checks.check("apogee within 9000–11000 ft", 9000 < apo < 11000, f"{apo:.0f} ft")


# ─── Binary-log assertions (pulled from the device over MTP) ──────────────────

def pull_and_decode(port: str | None, full: bool, checks: Checks,
                    keep: bool) -> None:
    print("\nPulling the binary flight log off the device (reflash MTP build)...")
    _pio_upload("teensy41_debug", None)   # teensy loader auto-finds via HID
    time.sleep(6)   # let MTP enumerate + storage_init run

    tmp = Path(tempfile.mkdtemp(prefix="apex_hil_selftest."))
    # libmtp opens a fresh session each call; retry once for enumeration races.
    listing = ""
    for _ in range(3):
        r = subprocess.run(["mtp-files"], capture_output=True, text=True)
        listing = r.stdout
        if "APXLOG" in listing:
            break
        time.sleep(3)
    # Parse "File ID: N\n   Filename: BOOT_xxxxx.APXLOG"
    pairs = []
    cur_id = None
    for line in listing.splitlines():
        s = line.strip()
        if s.startswith("File ID:"):
            cur_id = s.split(":", 1)[1].strip()
        elif s.startswith("Filename:") and cur_id is not None:
            name = s.split(":", 1)[1].strip()
            if name.endswith(".APXLOG"):
                pairs.append((cur_id, name))
            cur_id = None
    checks.check("device exposes APXLOG files over MTP", bool(pairs),
                 f"{len(pairs)} files")
    if not pairs:
        return

    # Both MTP storages (QSPI + SD) carry same-named BOOT_xxxxx.APXLOG files.
    # Namespace by File ID so the SD copy (intentionally missing BOOST/COAST
    # until the post-landing dump) does not overwrite the complete QSPI copy.
    # decode_files() then de-dups by (boot_id, seq), merging both into the union.
    pulled = []
    for fid, name in pairs:
        dst = tmp / f"{fid}_{name}"
        subprocess.run(["mtp-getfile", fid, str(dst)],
                       capture_output=True, text=True)
        if dst.exists() and dst.stat().st_size > 0:
            pulled.append(dst)

    # Decode each file INDIVIDUALLY rather than merging. The device carries two
    # storages (QSPI black box + SD mirror) whose files share BOOT_xxxxx names
    # and boot_ids; a merged decode de-dups by (boot_id, seq) and would collide
    # an old SD flight with the QSPI one if boot_ids ever repeat (e.g. after a
    # format). Per-file analysis picks the single newest COMPLETE flight log.
    best = None   # (boot_id, file, records, stats)
    any_bad_crc = 0
    for f in pulled:
        recs, st = decode_files([f])
        any_bad_crc += st.bad_crc
        s = [r.payload for r in recs if r.record_type == 3 and r.payload]
        ph = {p.get("phase") for p in s}
        if "BOOST" in ph and "COAST" in ph:           # a real ascent log
            boot = max((r.boot_id for r in recs), default=0)
            if best is None or boot > best[0]:
                best = (boot, f, recs, st)
    checks.check("at least one complete flight log on the device (BOOST+COAST)",
                 best is not None)
    checks.check("all decoded logs CRC-clean", any_bad_crc == 0,
                 f"bad_crc={any_bad_crc}")
    if best is None:
        return
    boot, _, frecs, _ = best
    fids = sorted({r.flight_id for r in frecs if r.flight_id})
    checks.check("flight_id assigned to the logged flight", bool(fids),
                 f"boot {boot} flight_ids={fids}")
    samples = [r.payload for r in frecs if r.record_type == 3 and r.payload]
    phases = {s.get("phase") for s in samples}
    events = [(r.payload or {}).get("event") for r in frecs if r.record_type == 2]

    # A real flight log: pad context (IDLE/ARMED) + BOOST + COAST present.
    checks.check("log has pad context (IDLE + ARMED samples)",
                 "IDLE" in phases and "ARMED" in phases, str(sorted(phases)))
    checks.check("log has BOOST + COAST samples",
                 "BOOST" in phases and "COAST" in phases)
    if full:
        checks.check("log has DESCENT + LANDED samples",
                     "DESCENT" in phases and "LANDED" in phases, str(sorted(phases)))

    coast = [s for s in samples if s.get("phase") == "COAST"]
    max_dep = max((s.get("deploy", 0.0) for s in coast), default=0.0)
    pid_live = any(abs(s.get("pid_p", 0.0)) > 0 for s in coast)
    checks.check("logged deployment ramps in COAST", max_dep > 0.3,
                 f"max deploy {max_dep*100:.0f}%")
    checks.check("logged PID terms populated", pid_live)
    faults = max((s.get("storage_faults", 0) for s in samples), default=0)
    checks.check("no storage faults during flight", faults == 0,
                 f"faults=0x{faults:04X}")

    if keep:
        print(f"\n  (kept pulled logs in {tmp})")
    else:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="Automated HIL flight-equivalence self-test")
    ap.add_argument("--port", default=None, help="serial port (default: auto)")
    ap.add_argument("--fake", action="store_true",
                    help="run against the in-process fake Teensy (no hardware)")
    ap.add_argument("--full", action="store_true",
                    help="fly through descent + landing (several minutes at 1x)")
    ap.add_argument("--flash", action="store_true",
                    help="flash teensy41_hil before the run")
    ap.add_argument("--pull", action="store_true",
                    help="after the flight, reflash debug + pull/decode the log")
    ap.add_argument("--keep", action="store_true", help="keep pulled log files")
    args = ap.parse_args()

    checks = Checks()
    fake = None
    try:
        if args.fake:
            from apex_sim.hil.fake_teensy import FakeTeensy
            fake = FakeTeensy()
            port = fake.port
            speed = 0.0
        else:
            if args.flash:
                _pio_upload("teensy41_hil", None)   # teensy loader auto-finds via HID
                time.sleep(2)
            port = args.port or _find_port(wait_s=15.0)
            if not port:
                print("No Teensy port found.", file=sys.stderr)
                return 2
            speed = 1.0

        env_cfg = load_environment()
        env = build_environment(env_cfg)
        rocket = build_rocket()
        ab = _load_airbrakes()

        print(f"Driving HIL flight ({'fake' if args.fake else port}, "
              f"{'full' if args.full else 'to apogee'})...")
        link = HilLink(port)
        try:
            if not args.fake:
                link.reset_input()
            if not link.wait_for_line("READY", timeout=15.0):
                print("No #HIL_READY — is teensy41_hil flashed?", file=sys.stderr)
                return 2
            result = run_closed_loop(
                link, env, rocket, env_cfg, airbrakes_cfg=ab,
                speed=speed, warmup_s=4.0,
                terminate_on_apogee=not args.full,
                max_time=600.0 if args.full else 120.0,
                noise=True)
        finally:
            link.close()

        print("\nIn-stream flight checks:")
        assert_flight(result, args.full, env_cfg.site.elevation_m, checks)

        if args.pull and not args.fake:
            pull_and_decode(port, args.full, checks, args.keep)
    finally:
        if fake is not None:
            fake.close()

    print(f"\n{'='*50}\n{checks.passed} passed, {checks.failed} failed\n{'='*50}")
    return 0 if checks.failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
