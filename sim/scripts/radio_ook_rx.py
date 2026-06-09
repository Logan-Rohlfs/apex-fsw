#!/usr/bin/env python3
"""
Decode the Apex RADIO_DATA_TEST OOK bench frame from RTL-SDR IQ samples.

The flight computer test frame is:
  preamble: 0x55 x 8
  sync:     0xD5
  payload:  "APEX RADIO TEST"
  checksum: XOR of payload bytes

Examples:
  # Live capture using the rtl_sdr CLI. Start this, then send RADIO_DATA_TEST.
  python scripts/radio_ook_rx.py --duration 14 --gain 10

  # Decode a previously captured unsigned 8-bit IQ file.
  python scripts/radio_ook_rx.py --iq-file capture.iq
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
DEFAULT_BIT_MS = 50.0
DEFAULT_PAYLOAD_LEN = len("APEX RADIO TEST")
MAX_PREAMBLE_ERRORS = 4
PREAMBLE = bytes([0x55] * 8)
SYNC = 0xD5


@dataclass
class DecodeResult:
    errors: int
    offset: int
    bit_index: int
    inverted: bool
    payload: bytes
    checksum_rx: int
    checksum_calc: int
    threshold: float

    @property
    def checksum_ok(self) -> bool:
        return self.checksum_rx == self.checksum_calc


def bytes_to_bits(data: bytes) -> list[int]:
    bits: list[int] = []
    for value in data:
        for bit in range(7, -1, -1):
            bits.append((value >> bit) & 1)
    return bits


def bits_to_byte(bits: np.ndarray) -> int:
    value = 0
    for bit in bits[:8]:
        value = (value << 1) | int(bit)
    return value


def load_u8_iq(raw: bytes) -> np.ndarray:
    u8 = np.frombuffer(raw, dtype=np.uint8)
    if len(u8) < 2:
        raise ValueError("IQ input is empty")
    if len(u8) % 2:
        u8 = u8[:-1]
    iq = u8.astype(np.float32).reshape(-1, 2) - 127.5
    return iq[:, 0] + 1j * iq[:, 1]


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


def envelope_bins(samples: np.ndarray, sample_rate: int, bin_hz: int = 1000) -> np.ndarray:
    samples_per_bin = max(1, int(sample_rate / bin_hz))
    usable = (len(samples) // samples_per_bin) * samples_per_bin
    if usable == 0:
        raise ValueError("not enough IQ samples")
    env = np.abs(samples[:usable])
    return env.reshape(-1, samples_per_bin).mean(axis=1)


def estimate_threshold(values: np.ndarray) -> float:
    lo, hi = np.percentile(values, [10, 90])
    return float((lo + hi) * 0.5)


def decode_bits(values: np.ndarray, samples_per_bit: int, offset: int, invert: bool) -> tuple[np.ndarray, float]:
    usable = len(values) - offset
    nbits = usable // samples_per_bit
    if nbits <= 0:
        return np.array([], dtype=np.uint8), 0.0

    bit_values = values[offset:offset + nbits * samples_per_bit]
    bit_values = bit_values.reshape(nbits, samples_per_bit).mean(axis=1)
    threshold = estimate_threshold(bit_values)
    bits = (bit_values > threshold).astype(np.uint8)
    if invert:
        bits = 1 - bits
    return bits, threshold


def find_best_frame(values: np.ndarray, samples_per_bit: int, payload_len: int) -> DecodeResult | None:
    header = np.array(bytes_to_bits(PREAMBLE + bytes([SYNC])), dtype=np.uint8)
    frame_bits = len(header) + (payload_len + 1) * 8
    best: DecodeResult | None = None

    for offset in range(samples_per_bit):
        for invert in (False, True):
            bits, threshold = decode_bits(values, samples_per_bit, offset, invert)
            if len(bits) < frame_bits:
                continue

            max_start = len(bits) - frame_bits
            for start in range(max_start + 1):
                errors = int(np.count_nonzero(bits[start:start + len(header)] != header))
                if best is not None and errors >= best.errors:
                    continue

                payload_start = start + len(header)
                payload = bytearray()
                for i in range(payload_len):
                    lo = payload_start + i * 8
                    payload.append(bits_to_byte(bits[lo:lo + 8]))

                cksum_index = payload_start + payload_len * 8
                checksum_rx = bits_to_byte(bits[cksum_index:cksum_index + 8])
                checksum_calc = 0
                for b in payload:
                    checksum_calc ^= b

                best = DecodeResult(
                    errors=errors,
                    offset=offset,
                    bit_index=start,
                    inverted=invert,
                    payload=bytes(payload),
                    checksum_rx=checksum_rx,
                    checksum_calc=checksum_calc,
                    threshold=threshold,
                )

    return best


def is_valid_frame(result: DecodeResult | None, max_preamble_errors: int = MAX_PREAMBLE_ERRORS) -> bool:
    return (
        result is not None
        and result.checksum_ok
        and result.errors <= max_preamble_errors
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode Apex RADIO_DATA_TEST OOK frames")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ_HZ)
    parser.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE_HZ)
    parser.add_argument("--duration", type=float, default=14.0)
    parser.add_argument("--gain", default="10", help='RTL gain in dB, or "auto"')
    parser.add_argument("--ppm", type=int, default=0)
    parser.add_argument("--device", default="0")
    parser.add_argument("--iq-file", help="unsigned 8-bit interleaved IQ file to decode")
    parser.add_argument("--save-iq", help="write live capture IQ bytes to this file")
    parser.add_argument("--bit-ms", type=float, default=DEFAULT_BIT_MS)
    parser.add_argument("--payload-len", type=int, default=DEFAULT_PAYLOAD_LEN)
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
    values = envelope_bins(samples, args.sample_rate)
    samples_per_bit = max(1, int(round(args.bit_ms)))
    result = find_best_frame(values, samples_per_bit, args.payload_len)

    if not is_valid_frame(result):
        if result is None:
            print("[rx] No frame found.")
        else:
            print(
                f"[rx] No valid frame. Best candidate: preamble_errors={result.errors} "
                f"checksum={'OK' if result.checksum_ok else 'BAD'}"
            )
        return 2

    try:
        text = result.payload.decode("ascii")
    except UnicodeDecodeError:
        text = result.payload.hex(" ")

    print(f"[rx] Preamble errors: {result.errors}")
    print(f"[rx] Bit offset: {result.offset} ms, bit index: {result.bit_index}, inverted={result.inverted}")
    print(f"[rx] Threshold: {result.threshold:.3f}")
    print(f"[rx] Payload: {text!r}")
    print(
        f"[rx] Checksum: rx=0x{result.checksum_rx:02X} calc=0x{result.checksum_calc:02X} "
        f"{'OK' if result.checksum_ok else 'BAD'}"
    )
    return 0 if result.checksum_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
