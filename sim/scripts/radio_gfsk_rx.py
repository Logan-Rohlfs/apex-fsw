#!/usr/bin/env python3
"""
Decode Apex 2-GFSK downlink frames from RTL-SDR IQ samples.

Wire format (must match fsw/src/radio.cpp):
  preamble: 0xAA x 8
  sync:     0x2D 0xD4
  type:     0x01 = RADIO_DATA_TEST (seq byte + "APEX RADIO TEST")
            0x02 = telemetry (TelemetryBody, little-endian, callsign first)
  crc:      CRC-16-CCITT (poly 0x1021, init 0xFFFF) over type+body, big-endian

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
import struct
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

FRAME_TYPE_TEST = 0x01
FRAME_TYPE_FLIGHT = 0x02
FRAME_TYPE_HK = 0x03

# fsw/src/radio.cpp TelemFlight / TelemHousekeeping (packed, little-endian).
# Scaled-int fields are converted back to SI here — keep in sync with the
# scale comments in radio.cpp.
FLIGHT_STRUCT = struct.Struct("<6sHBBbBff7hBHb")   # 38 bytes
HK_STRUCT = struct.Struct("<H3h3h2hH")             # 20 bytes
PHASE_NAMES = ("IDLE", "ARMED", "BOOST", "COAST", "DESCENT", "LANDED")

BODY_LEN_BY_TYPE = {
    FRAME_TYPE_TEST: 1 + PAYLOAD_LEN,        # seq + ASCII payload
    FRAME_TYPE_FLIGHT: FLIGHT_STRUCT.size,
    FRAME_TYPE_HK: HK_STRUCT.size,
}
MIN_BODY_LEN = min(BODY_LEN_BY_TYPE.values())


def parse_flight(body: bytes) -> dict:
    (callsign, seq, phase, health, gps_fix, gps_sats, lat, lon,
     gps_alt, alt, vel, apogee, vacc, accel_z, roll, deploy,
     baro2, btemp) = FLIGHT_STRUCT.unpack(body)
    return {
        "callsign": callsign.decode("ascii", "replace").strip("\x00 "),
        "seq": seq,
        "phase": phase,
        "phase_name": PHASE_NAMES[phase] if phase < len(PHASE_NAMES) else "UNKNOWN",
        "health": health,
        "gps_fix": gps_fix,
        "gps_sats": gps_sats,
        "gps_lat_deg": lat,
        "gps_lon_deg": lon,
        "gps_alt_msl_m": gps_alt * 0.5,
        "alt_agl_m": alt * 0.1,
        "velocity_mps": vel * 0.02,
        "pred_apogee_m": apogee * 0.1,
        "vert_accel_mps2": vacc * 0.01,
        "accel_z_mss": accel_z * 0.01,
        "roll_rate_rads": roll * 0.002,
        "deployment_frac": deploy / 255.0,
        "baro_pa": baro2 * 2.0,
        "baro_temp_c": float(btemp),
    }


def parse_housekeeping(body: bytes) -> dict:
    (seq, mx, my, mz, hx, hy, hz, gx, gy, up) = HK_STRUCT.unpack(body)
    return {
        "seq": seq,
        "mag_x_gauss": mx * 1e-4, "mag_y_gauss": my * 1e-4, "mag_z_gauss": mz * 1e-4,
        "highg_x_mss": hx * 0.1, "highg_y_mss": hy * 0.1, "highg_z_mss": hz * 0.1,
        "gyro_x_rads": gx * 0.002, "gyro_y_rads": gy * 0.002,
        "uptime_s": up,
    }

# Correlation template: last 4 preamble bytes + sync (48 bits). 0x2DD4 is
# DC-balanced, so the template mean doubles as the slicer threshold.
_TEMPLATE_BYTES = PREAMBLE[-4:] + SYNC

MIN_CORR_QUALITY = 0.45   # normalized correlation acceptance threshold
MAX_CANDIDATES = 4000     # bound on decode attempts per capture


@dataclass
class FrameResult:
    ftype: int              # FRAME_TYPE_TEST or FRAME_TYPE_TELEM
    seq: int
    payload: bytes          # body without the seq byte (test) / full body (telem)
    crc_rx: int
    crc_calc: int
    quality: float          # normalized preamble+sync correlation, 0..1
    freq_offset_hz: float   # carrier offset seen by the slicer
    sample_index: int       # template start in the capture

    @property
    def crc_ok(self) -> bool:
        return self.crc_rx == self.crc_calc

    def parse(self) -> dict:
        """Parse a FLIGHT or HOUSEKEEPING body into named SI-unit fields."""
        if self.ftype == FRAME_TYPE_FLIGHT:
            return parse_flight(self.payload)
        if self.ftype == FRAME_TYPE_HK:
            return parse_housekeeping(self.payload)
        raise ValueError(f"frame type 0x{self.ftype:02X} has no parser")


@dataclass
class DecodeStats:
    candidates: int = 0
    unknown_type: int = 0
    bad_crc: int = 0
    good_crc: int = 0
    best_quality: float = 0.0
    best_type: int | None = None
    best_crc_rx: int | None = None
    best_crc_calc: int | None = None


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
                max_frames: int = 128,
                stats: DecodeStats | None = None) -> list[FrameResult]:
    """Demodulate and return every decodable frame in the capture, in order."""
    sps = sample_rate // bitrate
    if sps < 2:
        raise ValueError("sample rate too low for bitrate")

    freq = quadrature_demod(samples)
    min_frame_bits = (1 + MIN_BODY_LEN + 2) * 8
    if len(freq) < sps * (len(_TEMPLATE_BYTES) * 8 + min_frame_bits):
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
    blocked = np.zeros(n_pos, dtype=bool)

    order = np.argsort(quality)[::-1][:MAX_CANDIDATES]
    for n0 in order:
        if len(results) >= max_frames:
            break
        if quality[n0] < MIN_CORR_QUALITY:
            break          # order is quality-descending — nothing better left
        if blocked[n0]:
            continue
        if stats is not None:
            stats.candidates += 1
            stats.best_quality = max(stats.best_quality, float(quality[n0]))

        # Slicer threshold = mean over the (DC-balanced) template region —
        # this is exactly the carrier frequency offset.
        tpl_idx = n0 + np.arange(tbits) * sps
        center = float(np.mean(smoothed[tpl_idx]))

        # Type byte first — it sets the frame length.
        type_idx = n0 + (tbits + np.arange(8)) * sps
        if type_idx[-1] >= len(smoothed):
            continue
        ftype = int(np.packbits((smoothed[type_idx] > center).astype(np.uint8))[0])
        if stats is not None and stats.best_type is None:
            stats.best_type = ftype
        body_len = BODY_LEN_BY_TYPE.get(ftype)
        if body_len is None:
            if stats is not None:
                stats.unknown_type += 1
            continue

        frame_bits = (1 + body_len + 2) * 8
        if n0 + (tbits + frame_bits) * sps > len(smoothed):
            continue

        bit_idx = n0 + (tbits + np.arange(frame_bits)) * sps
        bits = (smoothed[bit_idx] > center).astype(np.uint8)
        data = np.packbits(bits).tobytes()   # type + body + crc

        body = data[1:1 + body_len]
        crc_rx = (data[-2] << 8) | data[-1]
        crc_calc = crc16_ccitt(data[:1 + body_len])

        if ftype == FRAME_TYPE_TEST:
            seq, payload = body[0], body[1:]
        elif ftype == FRAME_TYPE_FLIGHT:
            seq, payload = struct.unpack_from("<H", body, 6)[0], body
        else:   # FRAME_TYPE_HK — seq leads the body
            seq, payload = struct.unpack_from("<H", body, 0)[0], body

        result = FrameResult(
            ftype=ftype,
            seq=seq,
            payload=payload,
            crc_rx=crc_rx,
            crc_calc=crc_calc,
            quality=float(quality[n0]),
            freq_offset_hz=center * sample_rate / (2.0 * np.pi),
            sample_index=int(n0),
        )
        if result.crc_ok:
            if stats is not None:
                stats.good_crc += 1
            results.append(result)
            frame_span = (tbits + frame_bits) * sps
            lo = max(0, n0 - frame_span)
            blocked[lo: min(n_pos, n0 + frame_span)] = True
        elif stats is not None:
            stats.bad_crc += 1
            if stats.best_crc_rx is None:
                stats.best_crc_rx = crc_rx
                stats.best_crc_calc = crc_calc

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
        if fr.ftype == FRAME_TYPE_FLIGHT:
            t = fr.parse()
            print(
                f"[rx] FLIGHT {t['callsign']} seq={t['seq']} phase={t['phase_name']} "
                f"alt={t['alt_agl_m']:.1f}m vel={t['velocity_mps']:.1f}m/s "
                f"apogee={t['pred_apogee_m']:.0f}m brake={t['deployment_frac']:.0%} "
                f"gps={t['gps_fix']}/{t['gps_sats']}sats "
                f"({t['gps_lat_deg']:.5f},{t['gps_lon_deg']:.5f}) "
                f"q={fr.quality:.2f} off={fr.freq_offset_hz / 1e3:+.2f}kHz"
            )
            continue
        if fr.ftype == FRAME_TYPE_HK:
            t = fr.parse()
            print(
                f"[rx] HK seq={t['seq']} mag=({t['mag_x_gauss']:.4f},{t['mag_y_gauss']:.4f},"
                f"{t['mag_z_gauss']:.4f})G highg=({t['highg_x_mss']:.1f},{t['highg_y_mss']:.1f},"
                f"{t['highg_z_mss']:.1f}) up={t['uptime_s']}s q={fr.quality:.2f}"
            )
            continue
        try:
            text = fr.payload.decode("ascii")
        except UnicodeDecodeError:
            text = fr.payload.hex(" ")
        print(
            f"[rx] TEST seq={fr.seq} payload={text!r} quality={fr.quality:.2f} "
            f"offset={fr.freq_offset_hz / 1e3:+.2f} kHz crc=OK"
        )

    print(f"[rx] {len(frames)} frame(s) decoded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
