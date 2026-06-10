#!/usr/bin/env python3
"""
Apex radio link diagnostic — answers "what is limiting us?"

Captures IQ from the RTL-SDR (or reads a saved file) and separates the link
into stages, so losses can be attributed:

  expected frames   — from --rate (firmware RADIO_TELEM_*_HZ)
  RF bursts seen    — power-envelope detection: the TX actually radiated and
                      the SDR heard it, regardless of whether it decoded
  frames decoded    — CRC-valid frames out of the GFSK decoder

  bursts < expected      → TX-side: firmware skips, or RF path loss so deep
                           the burst never rises above the noise floor
  decoded < bursts       → decode-side: low SNR, clipping, offset, or a bug
  decoded == expected    → the link is fine; look at the consumer

Also reports ADC clipping (gain/AGC overload), burst SNR, and carrier offset
spread — the usual silent killers.

All capture files live in a temporary folder that is deleted on exit
(pass --keep to retain it for further analysis).

Examples:
  # 10 s live capture while TELEM_ON at 10 Hz:
  python scripts/radio_diag.py --duration 10 --rate 10 --gain 10

  # Re-analyze a kept capture:
  python scripts/radio_diag.py --iq-file /path/capture.iq --rate 10
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from radio_gfsk_rx import (
    BITRATE_BPS,
    DEFAULT_FREQ_HZ,
    DEFAULT_SAMPLE_RATE_HZ,
    FRAME_TYPE_FLIGHT,
    FRAME_TYPE_HK,
    FRAME_TYPE_TEST,
    find_frames,
    load_u8_iq,
)

TYPE_NAMES = {FRAME_TYPE_TEST: "TEST", FRAME_TYPE_FLIGHT: "FLIGHT", FRAME_TYPE_HK: "HK"}

# Expected on-air durations (preamble..crc) in ms, ± tolerance for burst classing
AIRTIME_MS = {FRAME_TYPE_FLIGHT: 51 * 8 / BITRATE_BPS * 1e3,
              FRAME_TYPE_HK: 33 * 8 / BITRATE_BPS * 1e3,
              FRAME_TYPE_TEST: 29 * 8 / BITRATE_BPS * 1e3}


def capture(args, out_path: Path) -> bytes:
    rtl_sdr = shutil.which("rtl_sdr")
    if rtl_sdr is None:
        raise RuntimeError("rtl_sdr CLI not found. Install rtl-sdr or pass --iq-file.")
    cmd = [rtl_sdr, "-d", str(args.device), "-f", str(args.freq),
           "-s", str(args.sample_rate), "-n", str(int(args.sample_rate * args.duration)),
           "-p", str(args.ppm), "-b", "8192", str(out_path)]
    if args.gain.lower() != "auto":
        cmd[1:1] = ["-g", args.gain]
    print(f"[diag] capturing {args.duration:.0f}s at {args.freq/1e6:.3f} MHz "
          f"gain={args.gain} → {out_path}")
    proc = subprocess.run(cmd, stderr=subprocess.PIPE, check=False)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr.decode("utf-8", "replace"))
        raise RuntimeError(f"rtl_sdr exited {proc.returncode}")
    return out_path.read_bytes()


def detect_bursts(samples: np.ndarray, fs: int) -> tuple[list[tuple[float, float]], float]:
    """Return ([(start_s, duration_ms)], snr_db) from the power envelope."""
    win = max(1, fs // 1000)   # 1 ms smoothing
    power = np.abs(samples) ** 2
    csum = np.concatenate(([0.0], np.cumsum(power, dtype=np.float64)))
    env = (csum[win:] - csum[:-win]) / win

    floor = float(np.percentile(env, 20))
    peak = float(np.percentile(env, 99.5))
    if floor <= 0 or peak < 4 * floor:
        return [], 0.0
    thr = float(np.sqrt(floor * peak))

    mask = env > thr
    edges = np.flatnonzero(np.diff(mask.astype(np.int8)))
    if mask[0]:
        edges = np.concatenate(([0], edges))
    if mask[-1]:
        edges = np.concatenate((edges, [len(mask) - 1]))
    pairs = edges.reshape(-1, 2)

    # Merge gaps < 5 ms, drop blips < 5 ms
    merged = []
    for s, e in pairs:
        if merged and s - merged[-1][1] < 0.005 * fs:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    bursts = [(s / fs, (e - s) / fs * 1e3) for s, e in merged if (e - s) >= 0.005 * fs]

    if bursts:
        in_burst = np.zeros(len(env), dtype=bool)
        for s, e in merged:
            in_burst[s:e] = True
        snr_db = 10 * np.log10(float(np.mean(env[in_burst])) / floor)
    else:
        snr_db = 0.0
    return bursts, snr_db


def analyze(raw: bytes, fs: int, rate_hz: float, duration_s: float):
    u8 = np.frombuffer(raw, dtype=np.uint8)
    clip_pct = 100.0 * np.count_nonzero((u8 <= 1) | (u8 >= 254)) / len(u8)

    samples = load_u8_iq(raw)
    actual_s = len(samples) / fs
    bursts, snr_db = detect_bursts(samples, fs)
    frames = find_frames(samples, fs, max_frames=int(actual_s * rate_hz * 2) + 16)

    expected = int(rate_hz * actual_s)
    by_type: dict[int, int] = {}
    for f in frames:
        by_type[f.ftype] = by_type.get(f.ftype, 0) + 1
    offsets = np.array([f.freq_offset_hz for f in frames])
    qualities = np.array([f.quality for f in frames])

    # Seq continuity over decoded telemetry frames
    seqs = sorted(f.seq for f in frames if f.ftype in (FRAME_TYPE_FLIGHT, FRAME_TYPE_HK))
    seq_gaps = 0
    if len(seqs) >= 2:
        seq_gaps = (seqs[-1] - seqs[0] + 1) - len(set(seqs))

    print("\n──── capture ─────────────────────────────────────")
    print(f" samples            {len(samples):,}  ({actual_s:.1f} s @ {fs} S/s)")
    print(f" ADC clipping       {clip_pct:.2f}%"
          + ("   ← OVERLOAD: lower gain / avoid AGC (gain 0)" if clip_pct > 1.0 else ""))
    print("\n──── stages ──────────────────────────────────────")
    print(f" expected frames    ~{expected}   (--rate {rate_hz} Hz)")
    print(f" RF bursts seen     {len(bursts)}   (envelope detection, SNR {snr_db:.1f} dB)")
    print(f" frames decoded     {len(frames)}   "
          + " ".join(f"{TYPE_NAMES.get(t, hex(t))}={n}" for t, n in sorted(by_type.items())))
    print(f" seq gaps (telem)   {seq_gaps}   (frames the TX numbered but we never decoded)")

    if bursts:
        durs = np.array([d for _, d in bursts])
        print(f" burst durations    {np.median(durs):.1f} ms median "
              f"(FLIGHT≈{AIRTIME_MS[FRAME_TYPE_FLIGHT]:.0f}, HK≈{AIRTIME_MS[FRAME_TYPE_HK]:.0f})")
    if len(frames):
        print(f" decode quality     {qualities.mean():.2f} mean / {qualities.min():.2f} min")
        print(f" carrier offset     {offsets.mean()/1e3:+.2f} kHz mean, "
              f"{offsets.std():.0f} Hz spread")

    print("\n──── verdict ─────────────────────────────────────")
    if clip_pct > 1.0:
        print(" ADC is clipping — fix gain first, it corrupts everything downstream.")
    if expected and len(bursts) < 0.9 * expected:
        print(f" TX-side / deep RF loss: only {len(bursts)}/{expected} bursts reached the "
              "SDR.\n   → check firmware TELEM rate + skipped beats, antenna, range.")
    if bursts and len(frames) < 0.9 * len(bursts):
        print(f" Decode-side: {len(bursts)} bursts arrived but only {len(frames)} decoded."
              f"\n   → SNR {snr_db:.1f} dB"
              + (" is marginal (<12 dB); more gain/antenna." if snr_db < 12 else
                 " looks fine — check offset/clipping above."))
    if expected and len(frames) >= 0.9 * expected:
        print(" Link is healthy — decoded ≈ expected. Any loss seen in the monitor is "
              "downstream\n   (plot rate, dedupe), not the radio.")


def main() -> int:
    p = argparse.ArgumentParser(description="Apex radio link diagnostic")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--rate", type=float, default=10.0,
                   help="expected telemetry frame rate (config.h RADIO_TELEM_*_HZ)")
    p.add_argument("--freq", type=int, default=DEFAULT_FREQ_HZ)
    p.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE_HZ)
    p.add_argument("--gain", default="10")
    p.add_argument("--ppm", type=int, default=0)
    p.add_argument("--device", default="0")
    p.add_argument("--iq-file", help="analyze an existing capture instead of recording")
    p.add_argument("--keep", action="store_true",
                   help="keep the temp capture folder (prints its path)")
    args = p.parse_args()

    if args.iq_file:
        analyze(Path(args.iq_file).read_bytes(), args.sample_rate, args.rate, args.duration)
        return 0

    tmp_dir = Path(tempfile.mkdtemp(prefix="apex_radio_diag_"))
    try:
        raw = capture(args, tmp_dir / "capture.iq")
        analyze(raw, args.sample_rate, args.rate, args.duration)
        if args.keep:
            print(f"\n[diag] capture kept: {tmp_dir}/capture.iq")
        return 0
    finally:
        if not args.keep:
            shutil.rmtree(tmp_dir, ignore_errors=True)
            print(f"[diag] temp folder cleaned up: {tmp_dir}")


if __name__ == "__main__":
    raise SystemExit(main())
