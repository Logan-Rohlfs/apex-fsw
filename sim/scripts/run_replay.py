#!/usr/bin/env python3
"""
Apex HIL Flight Data Replay
============================
Plays back a Telemega CSV flight log to a Teensy running APEX_HIL firmware,
then displays live state machine transitions and deployment decisions.

Usage (from apex/sim/ with venv active):
    python scripts/run_replay.py --port /dev/cu.usbmodem<N>
    python scripts/run_replay.py  # auto-detects Teensy

Options:
    --port     Serial port (default: auto-detect)
    --csv      Telemega CSV (default: data/flights/seymour_2026_05_24/...)
    --pre-pad  Seconds of static on-pad idle to prepend (default: 5.0)
               Firmware needs ~3s warm-up (Mahony + baro buffer). 5s gives margin.
    --speed    Replay speed multiplier (default: 1.0, 0 = max speed)
    --out      CSV path for TeensyPacket log (default: output/hil_replay_<ts>.csv)

Body-frame convention (Seymour TX flight):
    accel_x ≈ +9.81 m/s² on pad (rocket vertical, X is axial / nose-up)
    gyro columns in deg/s → converted to rad/s for SimPacket

High-G note: Telemega has no separate ADXL375-class channel.
highg_* mirrors ICM accel. Firmware switches to high-G only above 14g
(peak in this flight: 14.10g at t=1.12s — brief, noted limitation).

Timing note: Teensy 4.1 USB CDC ignores HIL_BAUD. The port runs at USB 2.0
full-speed regardless. Do not diagnose timing issues by changing baud rate.
"""

from __future__ import annotations

import argparse
import csv as _csv
import math
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import NamedTuple

import numpy as np
import serial
import serial.tools.list_ports

# ─── Paths ────────────────────────────────────────────────────────────────────

_SIM_ROOT    = Path(__file__).resolve().parents[1]
_DEFAULT_CSV = (_SIM_ROOT / "data" / "flights" / "seymour_2026_05_24"
                / "telemega_flight_2026-05-24.csv")

# ─── Protocol (must match hil.h) ──────────────────────────────────────────────

HIL_MAGIC_SIM_TO_TEENSY = 0xABCD   # LE wire: [0xCD, 0xAB]
HIL_MAGIC_TEENSY_TO_SIM = 0xCDAB   # LE wire: [0xAB, 0xCD]
HIL_BAUD                = 921600

# SimPacket: magic(H) + sim_time_ms(I) + 14×float + gps_valid(B) + crc8(B) = 64 bytes
SIM_FMT  = "<HI" + "f" * 14 + "BB"
SIM_SIZE = struct.calcsize(SIM_FMT)
assert SIM_SIZE == 64, f"SimPacket={SIM_SIZE}"

# TeensyPacket: magic(H) + sim_time_ms(I) + 4×float + phase(B) + crc8(B) = 24 bytes
TEENSY_FMT  = "<HIffffBB"
TEENSY_SIZE = struct.calcsize(TEENSY_FMT)
assert TEENSY_SIZE == 24, f"TeensyPacket={TEENSY_SIZE}"

RATE_HZ = 100
DT_S    = 1.0 / RATE_HZ

PHASE_NAMES = {0: "IDLE", 1: "ARMED", 2: "BOOST", 3: "COAST", 4: "DESCENT", 5: "LANDED"}
PHASE_COLOR = {"IDLE": "\033[90m", "ARMED": "\033[34m", "BOOST": "\033[33m",
               "COAST": "\033[32m", "DESCENT": "\033[35m", "LANDED": "\033[36m"}
RESET = "\033[0m"; BOLD = "\033[1m"; RED = "\033[31m"; YEL = "\033[33m"; GRN = "\033[32m"

# ─── CRC-8 (poly 0x07, must match hil.cpp) ───────────────────────────────────

def _build_crc8_table() -> bytes:
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return bytes(t)

_CRC8 = _build_crc8_table()

def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = _CRC8[c ^ b]
    return c

# ─── Packet builders ──────────────────────────────────────────────────────────

def pack_sim(sim_ms: int, ax: float, ay: float, az: float,
             gx: float, gy: float, gz: float, baro: float,
             hx: float, hy: float, hz: float,
             mx: float, my: float, mz: float,
             gps_alt: float, gps_valid: int) -> bytes:
    body = struct.pack(SIM_FMT,
        HIL_MAGIC_SIM_TO_TEENSY, sim_ms,
        ax, ay, az, gx, gy, gz, baro,
        hx, hy, hz, mx, my, mz,
        gps_alt, gps_valid & 0xFF, 0)
    return body[:-1] + bytes([crc8(body[:-1])])

class TeensyPkt(NamedTuple):
    magic: int; sim_ms: int; deploy: float; alt: float; vel: float; pred: float
    phase: int; crc: int

def unpack_teensy(data: bytes) -> TeensyPkt | None:
    if len(data) < TEENSY_SIZE:
        return None
    f = struct.unpack_from(TEENSY_FMT, data)
    pkt = TeensyPkt(*f)
    if pkt.magic != HIL_MAGIC_TEENSY_TO_SIM:
        return None
    if crc8(data[:TEENSY_SIZE-1]) != pkt.crc:
        return None
    return pkt

# ─── CSV loading ──────────────────────────────────────────────────────────────

def load_telemega(path: Path) -> dict:
    """Load Telemega CSV into arrays. Returns dict of numpy arrays."""
    with open(path) as f:
        raw_hdr = f.readline().lstrip("#").strip()
    cols = [c.strip() for c in raw_hdr.split(",")]

    # Handle duplicate 'altitude' column: first = baro-derived MSL, second = GPS MSL
    alt_idxs = [i for i, c in enumerate(cols) if c == "altitude"]
    gps_alt_idx = alt_idxs[1] if len(alt_idxs) > 1 else cols.index("altitude")

    # Only read the specific numeric columns we need — avoids failing on
    # string columns (callsign, state_name, etc.) in the same row.
    numeric_cols = {
        "time":        cols.index("time"),
        "pressure":    cols.index("pressure"),
        "accel_x":     cols.index("accel_x"),
        "accel_y":     cols.index("accel_y"),
        "accel_z":     cols.index("accel_z"),
        "gyro_roll":   cols.index("gyro_roll"),
        "gyro_pitch":  cols.index("gyro_pitch"),
        "gyro_yaw":    cols.index("gyro_yaw"),
        "mag_x":       cols.index("mag_x"),
        "mag_y":       cols.index("mag_y"),
        "mag_z":       cols.index("mag_z"),
        "nsat":        cols.index("nsat"),
        "gps_alt_msl": gps_alt_idx,
    }
    state_name_idx = cols.index("state_name")

    data: dict = {k: [] for k in numeric_cols}
    data["state_name"] = []

    with open(path, newline="") as f:
        f.readline()  # skip header
        for row in _csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            if len(row) <= max(numeric_cols.values()):
                continue
            try:
                for k, i in numeric_cols.items():
                    v = row[i].strip()
                    data[k].append(float(v) if v else 0.0)
                data["state_name"].append(row[state_name_idx].strip() if len(row) > state_name_idx else "")
            except (ValueError, IndexError):
                continue

    if not data["time"]:
        raise ValueError(f"No data in {path}")

    for k in numeric_cols:
        data[k] = np.array(data[k], dtype=np.float64)

    return {
        "time":         data["time"],
        "pressure":     data["pressure"],
        "accel_x":      data["accel_x"],
        "accel_y":      data["accel_y"],
        "accel_z":      data["accel_z"],
        "gyro_roll":    data["gyro_roll"]  * math.pi / 180.0,
        "gyro_pitch":   data["gyro_pitch"] * math.pi / 180.0,
        "gyro_yaw":     data["gyro_yaw"]   * math.pi / 180.0,
        "mag_x":        data["mag_x"],
        "mag_y":        data["mag_y"],
        "mag_z":        data["mag_z"],
        "nsat":         data["nsat"],
        "gps_alt_msl":  data["gps_alt_msl"],
        "state_name":   data["state_name"],
    }

def build_grid(data: dict, pre_pad_s: float) -> dict:
    """
    Build a uniform 100 Hz grid using zero-order hold (forward-fill).
    ZOH preserves discontinuities like the burnout accel step — critical for
    testing state machine transitions. Interpolation would soften them.
    Pre-pads `pre_pad_s` seconds of the first row (rocket on pad, stationary).
    """
    t_orig  = data["time"]
    t_start = t_orig[0] - pre_pad_s
    t_end   = t_orig[-1]
    t_grid  = np.arange(t_start, t_end + DT_S * 0.5, DT_S)

    # For each grid point, find the last original sample at or before it.
    # clip to [0, n-1] so pre-pad region maps to first row.
    idx = np.clip(np.searchsorted(t_orig, t_grid, side="right") - 1, 0, len(t_orig) - 1)

    out: dict = {"t_grid": t_grid, "sim_ms": np.round((t_grid - t_start) * 1000).astype(np.uint32)}
    for k, v in data.items():
        if k == "state_name":
            out[k] = [v[i] for i in idx]
        elif isinstance(v, np.ndarray):
            out[k] = v[idx]
    return out

# ─── Async TeensyPacket receiver ─────────────────────────────────────────────

class TeensyReceiver(threading.Thread):
    """Background thread that parses TeensyPackets from the serial stream."""

    MAGIC_B0 = HIL_MAGIC_TEENSY_TO_SIM & 0xFF         # 0xAB
    MAGIC_B1 = (HIL_MAGIC_TEENSY_TO_SIM >> 8) & 0xFF  # 0xCD

    def __init__(self, port: serial.Serial):
        super().__init__(daemon=True)
        self._port    = port
        self._lock    = threading.Lock()
        self._pkts:   list[TeensyPkt] = []
        self._log:    list[str]       = []
        self.last:    TeensyPkt | None = None
        self._run     = True

    def stop(self): self._run = False

    def get_pkts(self) -> list[TeensyPkt]:
        with self._lock: return list(self._pkts)

    def get_log(self) -> list[str]:
        with self._lock: return list(self._log)

    def run(self):
        buf  = bytearray()
        syn  = False
        while self._run:
            try:
                raw = self._port.read(1)
            except Exception:
                break
            if not raw:
                continue
            b = raw[0]
            if not syn:
                buf.append(b)
                if len(buf) >= 2 and buf[-2] == self.MAGIC_B0 and buf[-1] == self.MAGIC_B1:
                    syn = True
                    buf = bytearray(buf[-2:])
                elif b == ord('\n') and buf:
                    line = buf.decode("utf-8", errors="replace").rstrip("\r\n")
                    buf.clear()
                    with self._lock: self._log.append(line)
                continue
            buf.append(b)
            if len(buf) < TEENSY_SIZE:
                continue
            pkt = unpack_teensy(bytes(buf))
            buf.clear(); syn = False
            if pkt:
                with self._lock:
                    self._pkts.append(pkt)
                    self.last = pkt

# ─── Port auto-detection ──────────────────────────────────────────────────────

TEENSY_VID = 0x16C0

def find_port() -> str | None:
    ports = list(serial.tools.list_ports.comports())
    def score(p):
        if p.vid == TEENSY_VID: return 3
        d = p.device.lower()
        if "usbmodem" in d or "acm" in d: return 2
        if "usb" in d: return 1
        return 0
    best = max(ports, key=score, default=None)
    return best.device if best and score(best) > 0 else None

# ─── Main replay ──────────────────────────────────────────────────────────────

def run_replay(port_name: str, csv_path: Path, pre_pad_s: float = 5.0,
               speed: float = 1.0, out_path: Path | None = None):

    print(f"Loading {csv_path.name}")
    raw   = load_telemega(csv_path)
    grid  = build_grid(raw, pre_pad_s)
    n     = len(grid["t_grid"])
    print(f"  {n} packets  |  {grid['t_grid'][0]:.2f}s – {grid['t_grid'][-1]:.2f}s  "
          f"|  pre-pad {pre_pad_s:.1f}s")

    print(f"\nConnecting to {port_name}")
    ser = serial.Serial(port_name, HIL_BAUD, timeout=0.1)
    time.sleep(0.3); ser.reset_input_buffer()

    print("Waiting for #HIL_READY...")
    deadline = time.monotonic() + 15
    ready = False
    buf = ""
    while time.monotonic() < deadline:
        buf += ser.read(128).decode("ascii", errors="replace")
        if "#HIL_READY" in buf:
            ready = True; break
        time.sleep(0.05)
    if not ready:
        print(f"{RED}ERROR: No #HIL_READY within 15s. Check firmware (-DAPEX_HIL) and USB.{RESET}")
        ser.close(); sys.exit(1)
    print(f"{GRN}Teensy ready.{RESET}")

    rx = TeensyReceiver(ser); rx.start()

    if out_path is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = _SIM_ROOT / "output" / f"hil_replay_{ts}.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"\n{'t(s)':>7}  {'CSV':^8}  {'Phase':^10}  {'AltAGL':>8}  {'Vel':>7}  "
          f"{'PredApo':>9}  {'Deploy':>7}  {'Lat(ms)':>8}")
    print("-" * 72)

    prev_phase = ""; sent = 0
    wall0 = time.monotonic(); next_dl = wall0

    class Summary:
        sent = 0; rcvd = 0; max_alt = 0.0; max_vel = 0.0
        transitions: list = []; first_deploy: float | None = None; lats: list = []

    S = Summary()

    for i in range(n):
        t_s    = float(grid["t_grid"][i])
        sim_ms = int(grid["sim_ms"][i])
        state  = grid["state_name"][i] if i < len(grid["state_name"]) else "?"

        ax = float(grid["accel_x"][i]); ay = float(grid["accel_y"][i]); az = float(grid["accel_z"][i])
        gx = float(grid["gyro_roll"][i]); gy = float(grid["gyro_pitch"][i]); gz = float(grid["gyro_yaw"][i])
        bp = float(grid["pressure"][i])
        mx = float(grid["mag_x"][i]); my = float(grid["mag_y"][i]); mz = float(grid["mag_z"][i])
        nsat = int(round(float(grid["nsat"][i])))
        gv = 1 if nsat >= 4 else 0
        ga = float(grid["gps_alt_msl"][i]) if gv else float("nan")

        pkt = pack_sim(sim_ms, ax, ay, az, gx, gy, gz, bp, ax, ay, az, mx, my, mz, ga, gv)
        t_send = time.monotonic()
        ser.write(pkt); S.sent += 1

        # Wait up to 50ms for response (5x tick period — handles OS jitter)
        pkt_rcvd = None
        deadline = t_send + 0.05
        while pkt_rcvd is None and time.monotonic() < deadline:
            pkts = rx.get_pkts()
            if len(pkts) > S.rcvd:
                pkt_rcvd = pkts[S.rcvd]
                S.rcvd += 1
            else:
                time.sleep(0.001)

        lat_ms = (time.monotonic() - t_send) * 1000

        if pkt_rcvd:
            ph = PHASE_NAMES.get(pkt_rcvd.phase, f"UNK({pkt_rcvd.phase})")
            S.lats.append(lat_ms)
            if pkt_rcvd.alt > S.max_alt: S.max_alt = pkt_rcvd.alt
            if pkt_rcvd.vel > S.max_vel: S.max_vel = pkt_rcvd.vel
            if pkt_rcvd.deploy > 0.005 and S.first_deploy is None: S.first_deploy = t_s

            phase_chg = ph != prev_phase and prev_phase != ""
            if phase_chg: S.transitions.append((t_s, ph))

            if i % 10 == 0 or phase_chg or (S.first_deploy is not None and abs(t_s - S.first_deploy) < DT_S * 2):
                c = PHASE_COLOR.get(ph, "")
                hi = f"{BOLD}{YEL}>>> TRANSITION {prev_phase}→{ph}  " if phase_chg else ""
                hi += f"{GRN}DEPLOY  " if S.first_deploy is not None and abs(t_s - S.first_deploy) < DT_S * 2 else ""
                print(f"{hi}{t_s:7.2f}  {state:^8}  {c}{ph:^10}{RESET}  "
                      f"{pkt_rcvd.alt:8.1f}  {pkt_rcvd.vel:7.2f}  "
                      f"{pkt_rcvd.pred:9.1f}  {pkt_rcvd.deploy*100:6.1f}%  {lat_ms:7.1f}")
            prev_phase = ph
        elif i % 50 == 0:
            print(f"{RED}  t={t_s:.2f}: no response (lat={lat_ms:.0f}ms){RESET}")

        # Deadline-based pacing — accumulated drift stays near zero
        next_dl += DT_S / max(speed, 0.001) if speed > 0 else 0
        rem = next_dl - time.monotonic()
        if rem > 0.0005: time.sleep(rem)

    time.sleep(0.5); rx.stop()

    # ── Save log ──────────────────────────────────────────────────────────────
    pkts = rx.get_pkts()
    if pkts:
        fields = ["sim_ms","deploy","alt","vel","pred","phase"]
        with open(out_path, "w", newline="") as f:
            w = _csv.writer(f)
            w.writerow(fields)
            for p in pkts:
                w.writerow([p.sim_ms, f"{p.deploy:.4f}", f"{p.alt:.2f}",
                             f"{p.vel:.3f}", f"{p.pred:.1f}", PHASE_NAMES.get(p.phase, p.phase)])
        print(f"\nLog → {out_path}  ({len(pkts)} packets)")

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*55}")
    print(f"{BOLD}Replay Summary{RESET}")
    print(f"  Sent: {S.sent}   Received: {S.rcvd}")
    if S.lats:
        sl = sorted(S.lats)
        print(f"  Latency: mean={sum(sl)/len(sl):.1f}ms  p95={sl[int(len(sl)*.95)]:.1f}ms  max={sl[-1]:.1f}ms")
    print(f"  Max alt AGL:  {S.max_alt:.1f} m")
    print(f"  Max velocity: {S.max_vel:.1f} m/s")
    if S.first_deploy is not None:
        print(f"{GRN}  First deployment at t={S.first_deploy:.2f}s{RESET}")
    else:
        print(f"  Deployment: none triggered")
    print(f"  Phase transitions:")
    for t, p in S.transitions:
        c = PHASE_COLOR.get(p, "")
        print(f"    t={t:7.2f}s  →  {c}{p}{RESET}")
    print("=" * 55)
    ser.close()

# ─── CLI ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Apex HIL flight data replay")
    ap.add_argument("--port",    default=None)
    ap.add_argument("--csv",     default=str(_DEFAULT_CSV))
    ap.add_argument("--pre-pad", type=float, default=5.0,
                    help="Seconds of on-pad idle before CSV start (default: 5.0)")
    ap.add_argument("--speed",   type=float, default=1.0,
                    help="Replay speed multiplier (default: 1.0, 0=max)")
    ap.add_argument("--out",     default=None)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print(f"{RED}No Teensy found. Specify --port.{RESET}"); sys.exit(1)
    print(f"Port: {port}")

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"{RED}CSV not found: {csv_path}{RESET}"); sys.exit(1)

    run_replay(port, csv_path, args.pre_pad, args.speed,
               Path(args.out) if args.out else None)

if __name__ == "__main__":
    main()
