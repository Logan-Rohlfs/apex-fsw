#!/usr/bin/env python3
"""
Decode the Apex RADIO_DATA_TEST 2-GFSK frames from RTL-SDR IQ samples.

Wire format (must match fsw/src/radio.cpp radio_data_test_tx):
  preamble: 0xAA x 8
  sync:     0x2D 0xD4
  seq:      1 byte (1..N within a test burst)
  payload:  "APEX RADIO TEST"
  crc:      CRC-16-CCITT (poly 0x1021, init 0xFFFF) over seq+payload, big-endian

Modulation: 2-GFSK, 10 kbps, ±25 kHz deviation, MSB-first, bit 1 = +deviation.

Demodulation is a single vectorized pass — quadrature discriminator, boxcar
bit filter, preamble+sync correlation, CRC check — so it runs comfortably in
real time on a rolling capture buffer (unlike the old OOK envelope search).

Examples:
  # Live capture using the rtl_sdr CLI. Start this, then send RADIO_DATA_TEST.
  python scripts/radio_gfsk_rx.py --duration 4 --gain 10

  # Decode a previously captured unsigned 8-bit IQ file.
  python scripts/radio_gfsk_rx.py --iq-file capture.iq
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass

import numpy as np


DEFAULT_FREQ_HZ = 441_480_000
DEFAULT_SAMPLE_RATE_HZ = 240_000
BITRATE_BPS = 10_000
DEVIATION_HZ = 25_000

PREAMBLE = bytes([0xAA] * 8)
SYNC = bytes([0x2D, 0xD4])
PAYLOAD = b"APEX RADIO TEST"
PAYLOAD_LEN = len(PAYLOAD)
BODY_LEN = 1 + PAYLOAD_LEN          # seq + payload
FRAME_BITS_AFTER_SYNC = (BODY_LEN + 2) * 8

# Correlation template: last 4 preamble bytes + sync (48 bits). 0x2DD4 is
# DC-balanced, so the template mean doubles as the slicer threshold.
_TEMPLATE_BYTES = PREAMBLE[-4:] + SYNC

MIN_CORR_QUALITY = 0.45   # normalized correlation acceptance threshold
MAX_CANDIDATES = 2000     # bound on decode attempts per capture


@dataclass
class FrameResult:
    seq: int
    payload: bytes
    crc_rx: int
    crc_calc: int
    quality: float          # normalized preamble+sync correlation, 0..1
    freq_offset_hz: float   # carrier offset seen by the slicer
    sample_index: int       # template start in the capture

    @property
    def crc_ok(self) -> bool:
        return self.crc_rx == self.crc_calc


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        crc &= 0xFFFF
    return crc


def bytes_to_nrz(data: bytes) -> np.ndarray:
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))
    return bits.astype(np.float32) * 2.0 - 1.0


def load_u8_iq(raw: bytes) -> np.ndarray:
    u8 = np.frombuffer(raw, dtype=np.uint8)
    if len(u8) < 2:
        raise ValueError("IQ input is empty")
    if len(u8) % 2:
        u8 = u8[:-1]
    iq = u8.astype(np.float32).reshape(-1, 2) - 127.5
    return iq[:, 0] + 1j * iq[:, 1]


def quadrature_demod(samples: np.ndarray) -> np.ndarray:
    """Instantaneous frequency in radians/sample."""
    return np.angle(samples[1:] * np.conj(samples[:-1]))


def find_frames(samples: np.ndarray, sample_rate: int,
                bitrate: int = BITRATE_BPS,
                max_frames: int = 32) -> list[FrameResult]:
    """Demodulate and return every decodable frame in the capture, in order."""
    sps = sample_rate // bitrate
    if sps < 2:
        raise ValueError("sample rate too low for bitrate")

    freq = quadrature_demod(samples)
    if len(freq) < sps * (len(_TEMPLATE_BYTES) * 8 + FRAME_BITS_AFTER_SYNC):
        return []

    # Bit matched filter: boxcar average over one bit period.
    # smoothed[i] = mean(freq[i : i+sps])
    csum = np.cumsum(freq, dtype=np.float64)
    csum = np.concatenate(([0.0], csum))
    smoothed = ((csum[sps:] - csum[:-sps]) / sps).astype(np.float32)

    # Correlate the ±1 template at bit spacing: corr[n] = Σ t[k]·smoothed[n+k·sps]
    template = bytes_to_nrz(_TEMPLATE_BYTES)
    tbits = len(template)
    n_pos = len(smoothed) - (tbits - 1) * sps
    if n_pos <= 0:
        return []
    corr = np.zeros(n_pos, dtype=np.float32)
    for k in range(tbits):
        corr += template[k] * smoothed[k * sps: k * sps + n_pos]

    # Normalize by deviation so quality ≈ 1.0 for a clean frame
    dev_rad = 2.0 * np.pi * DEVIATION_HZ / sample_rate
    quality = corr / (tbits * dev_rad)

    results: list[FrameResult] = []
    frame_span = (tbits + FRAME_BITS_AFTER_SYNC) * sps
    blocked = np.zeros(n_pos, dtype=bool)

    order = np.argsort(quality)[::-1][:MAX_CANDIDATES]
    for n0 in order:
        if len(results) >= max_frames:
            break
        if quality[n0] < MIN_CORR_QUALITY:
            break          # order is quality-descending — nothing better left
        if blocked[n0]:
            continue
        if n0 + frame_span > len(smoothed):
            continue

        # Slicer threshold = mean over the (DC-balanced) template region —
        # this is exactly the carrier frequency offset.
        tpl_idx = n0 + np.arange(tbits) * sps
        center = float(np.mean(smoothed[tpl_idx]))

        bit_idx = n0 + (tbits + np.arange(FRAME_BITS_AFTER_SYNC)) * sps
        bits = (smoothed[bit_idx] > center).astype(np.uint8)
        data = np.packbits(bits).tobytes()

        body, crc_bytes = data[:BODY_LEN], data[BODY_LEN:BODY_LEN + 2]
        crc_rx = (crc_bytes[0] << 8) | crc_bytes[1]
        crc_calc = crc16_ccitt(body)

        result = FrameResult(
            seq=body[0],
            payload=body[1:],
            crc_rx=crc_rx,
            crc_calc=crc_calc,
            quality=float(quality[n0]),
            freq_offset_hz=center * sample_rate / (2.0 * np.pi),
            sample_index=int(n0),
        )
        if result.crc_ok:
            results.append(result)
            lo = max(0, n0 - frame_span)
            blocked[lo: min(n_pos, n0 + frame_span)] = True

    results.sort(key=lambda r: r.sample_index)
    return results


def capture_rtl_sdr(args: argparse.Namespace) -> bytes:
    rtl_sdr = shutil.which("rtl_sdr")
    if rtl_sdr is None:
        raise RuntimeError("rtl_sdr CLI not found. Install rtl-sdr or pass --iq-file.")

    sample_count = int(args.sample_rate * args.duration)
    cmd = [
        rtl_sdr,
        "-d", str(args.device),
        "-f", str(args.freq),
        "-s", str(args.sample_rate),
        "-n", str(sample_count),
        "-p", str(args.ppm),
        "-",
    ]
    if args.gain.lower() != "auto":
        cmd[1:1] = ["-g", args.gain]

    print(
        f"[rx] Capturing {args.duration:.1f}s at {args.freq / 1e6:.6f} MHz, "
        f"sample_rate={args.sample_rate}, gain={args.gain}"
    )
    print("[rx] Send RADIO_DATA_TEST now if it is not already running.")

    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr.decode("utf-8", errors="replace"))
        raise RuntimeError(f"rtl_sdr exited with status {proc.returncode}")
    return proc.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode Apex RADIO_DATA_TEST 2-GFSK frames")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ_HZ)
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE_HZ)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--gain", default="10", help='RTL gain in dB, or "auto"')
    parser.add_argument("--ppm", type=int, default=0)
    parser.add_argument("--device", default="0")
    parser.add_argument("--iq-file", help="unsigned 8-bit interleaved IQ file to decode")
    parser.add_argument("--save-iq", help="write live capture IQ bytes to this file")
    args = parser.parse_args()

    if args.iq_file:
        with open(args.iq_file, "rb") as f:
            raw = f.read()
    else:
        raw = capture_rtl_sdr(args)

    if args.save_iq:
        with open(args.save_iq, "wb") as f:
            f.write(raw)

    samples = load_u8_iq(raw)
    frames = find_frames(samples, args.sample_rate)

    if not frames:
        print("[rx] No valid frames found.")
        return 2

    for fr in frames:
        try:
            text = fr.payload.decode("ascii")
        except UnicodeDecodeError:
            text = fr.payload.hex(" ")
        print(
            f"[rx] seq={fr.seq} payload={text!r} quality={fr.quality:.2f} "
            f"offset={fr.freq_offset_hz / 1e3:+.2f} kHz crc=OK"
        )

    print(f"[rx] {len(frames)} frame(s) decoded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
