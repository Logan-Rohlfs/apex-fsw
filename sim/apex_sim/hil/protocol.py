"""HIL serial wire protocol — Python mirror of ``fsw/src/hil.h``.

This module is the single source of truth on the Python side.  The C side is
``fsw/src/hil.h`` (``SimPacket`` / ``TeensyPacket``).  **Both must change in
the same commit**, with the ``static_assert`` sizes on the C side and the
``*_SIZE`` asserts below kept in lockstep.

Wire format
-----------
Little-endian packed structs, CRC-8 (poly 0x07, CRC-8/SMBUS) over every byte
except the trailing ``crc8`` field itself.

Sim → Teensy (64 bytes)::

    uint16 magic = 0xABCD     (LE wire bytes: CD AB)
    uint32 sim_time_ms
    float  accel_x/y/z_mss    ICM body frame
    float  gyro_x/y/z_rads    ICM body frame
    float  baro_pa
    float  highg_x/y/z_mss    ADXL375 body frame
    float  mag_x/y/z_gauss    MMC5983MA body frame
    float  gps_alt_msl_m      NaN = no fix
    uint8  gps_valid          bit0 = GPS fix, bit1 = arm switches closed
    uint8  crc8

Teensy → Sim (24 bytes)::

    uint16 magic = 0xCDAB     (LE wire bytes: AB CD)
    uint32 sim_time_ms        echo for latency measurement
    float  deployment_frac    0.0–1.0
    float  est_alt_agl_m
    float  est_vel_mps
    float  pred_apogee_m
    uint8  phase              FlightPhase enum value
    uint8  crc8
"""

from __future__ import annotations

import struct
from typing import NamedTuple, Optional

import serial.tools.list_ports

# ─── Constants (mirror hil.h) ─────────────────────────────────────────────────

HIL_MAGIC_SIM_TO_TEENSY = 0xABCD   # LE wire: [0xCD, 0xAB]
HIL_MAGIC_TEENSY_TO_SIM = 0xCDAB   # LE wire: [0xAB, 0xCD]
HIL_BAUD = 921600                  # ignored by Teensy USB CDC, required by pyserial
RATE_HIL_HZ = 100
DT_S = 1.0 / RATE_HIL_HZ

# gps_valid is a bitfield (mirror hil.h): bit0 = GPS fix, bit1 = arm switches
# closed. Packed into one byte so the wire stays 64 B.
HIL_GPS_FIX_BIT = 0x01
HIL_ARM_SWITCH_BIT = 0x02

SIM_STRUCT = struct.Struct("<HI" + "f" * 14 + "BB")
SIM_SIZE = SIM_STRUCT.size
assert SIM_SIZE == 64, "SimPacket size mismatch — sync with fsw/src/hil.h"

TEENSY_STRUCT = struct.Struct("<HIffffBB")
TEENSY_SIZE = TEENSY_STRUCT.size
assert TEENSY_SIZE == 24, "TeensyPacket size mismatch — sync with fsw/src/hil.h"

PHASE_NAMES = {0: "IDLE", 1: "ARMED", 2: "BOOST", 3: "COAST", 4: "DESCENT", 5: "LANDED"}

# ─── CRC-8 (poly 0x07, init 0x00 — must match hil.cpp) ───────────────────────


def _build_crc8_table() -> bytes:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return bytes(table)


_CRC8_TABLE = _build_crc8_table()


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS over ``data`` (poly 0x07, init 0x00)."""
    c = 0
    for b in data:
        c = _CRC8_TABLE[c ^ b]
    return c


# ─── Packet types ─────────────────────────────────────────────────────────────


class SimSensors(NamedTuple):
    """Sensor fields of a SimPacket (everything between magic and crc8)."""

    accel_x_mss: float
    accel_y_mss: float
    accel_z_mss: float
    gyro_x_rads: float
    gyro_y_rads: float
    gyro_z_rads: float
    baro_pa: float
    highg_x_mss: float
    highg_y_mss: float
    highg_z_mss: float
    mag_x_gauss: float
    mag_y_gauss: float
    mag_z_gauss: float
    gps_alt_msl_m: float
    gps_valid: int
    arm_switch: int = 0   # 1 = operator arm switches closed (HIL_ARM_SWITCH_BIT)


class TeensyPkt(NamedTuple):
    magic: int
    sim_time_ms: int
    deployment_frac: float
    est_alt_agl_m: float
    est_vel_mps: float
    pred_apogee_m: float
    phase: int
    crc: int


# ─── Pack / unpack ────────────────────────────────────────────────────────────


def pack_sim(sim_time_ms: int, sensors: SimSensors) -> bytes:
    """Serialise a SimPacket with a valid CRC-8 trailer.

    gps_valid + arm_switch are combined into the single gps_valid wire byte
    (bit0 = fix, bit1 = arm switches closed)."""
    gps_byte = ((HIL_GPS_FIX_BIT if sensors.gps_valid else 0)
                | (HIL_ARM_SWITCH_BIT if sensors.arm_switch else 0))
    body = SIM_STRUCT.pack(
        HIL_MAGIC_SIM_TO_TEENSY, sim_time_ms & 0xFFFFFFFF,
        *sensors[:14], gps_byte & 0xFF, 0)
    return body[:-1] + bytes([crc8(body[:-1])])


def unpack_sim(data: bytes) -> Optional["tuple"]:
    """Parse a SimPacket. Returns ``(sim_time_ms, SimSensors)`` or None."""
    if len(data) < SIM_SIZE:
        return None
    fields = SIM_STRUCT.unpack_from(data)
    if fields[0] != HIL_MAGIC_SIM_TO_TEENSY:
        return None
    if crc8(data[:SIM_SIZE - 1]) != fields[-1]:
        return None
    gps_byte = fields[16]
    return fields[1], SimSensors(
        *fields[2:16],
        gps_valid=1 if gps_byte & HIL_GPS_FIX_BIT else 0,
        arm_switch=1 if gps_byte & HIL_ARM_SWITCH_BIT else 0)


def pack_teensy(pkt: TeensyPkt) -> bytes:
    """Serialise a TeensyPacket with a valid CRC-8 trailer (fake-Teensy use)."""
    body = TEENSY_STRUCT.pack(
        HIL_MAGIC_TEENSY_TO_SIM, pkt.sim_time_ms & 0xFFFFFFFF,
        pkt.deployment_frac, pkt.est_alt_agl_m, pkt.est_vel_mps,
        pkt.pred_apogee_m, pkt.phase & 0xFF, 0)
    return body[:-1] + bytes([crc8(body[:-1])])


def unpack_teensy(data: bytes) -> Optional[TeensyPkt]:
    """Parse a TeensyPacket. Returns None on bad magic/CRC/length."""
    if len(data) < TEENSY_SIZE:
        return None
    pkt = TeensyPkt(*TEENSY_STRUCT.unpack_from(data))
    if pkt.magic != HIL_MAGIC_TEENSY_TO_SIM:
        return None
    if crc8(data[:TEENSY_SIZE - 1]) != pkt.crc:
        return None
    return pkt


# ─── Port auto-detection ──────────────────────────────────────────────────────

TEENSY_VID = 0x16C0


def find_port() -> Optional[str]:
    """Best-guess Teensy serial port, or None if nothing plausible is present."""
    ports = list(serial.tools.list_ports.comports())

    def score(p):
        if p.vid == TEENSY_VID:
            return 3
        d = p.device.lower()
        if "usbmodem" in d or "acm" in d:
            return 2
        if "usb" in d:
            return 1
        return 0

    best = max(ports, key=score, default=None)
    return best.device if best and score(best) > 0 else None
