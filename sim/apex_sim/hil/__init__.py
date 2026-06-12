"""HIL serial protocol and closed-loop runner — Python side.

Wire format is defined by ``fsw/src/hil.h`` and mirrored in
:mod:`apex_sim.hil.protocol` — change both in the same commit.

Modules
-------
protocol     packet structs, CRC-8, pack/unpack, Teensy port detection
link         HilLink — threaded serial link (packets + ASCII status lines)
emulator     SensorEmulator — RocketPy true state → SimPacket sensor fields
fake_teensy  FakeTeensy — pty-backed firmware reference, for hardware-free runs
runner       warm_up / run_closed_loop — the real-time HIL loop

Entry point: ``scripts/run_hil.py``.  Replay of recorded flights:
``scripts/run_replay.py``.
"""

from apex_sim.hil.protocol import (
    HIL_BAUD,
    PHASE_NAMES,
    SimSensors,
    TeensyPkt,
    crc8,
    find_port,
    pack_sim,
    pack_teensy,
    unpack_sim,
    unpack_teensy,
)

__all__ = [
    "HIL_BAUD",
    "PHASE_NAMES",
    "SimSensors",
    "TeensyPkt",
    "crc8",
    "find_port",
    "pack_sim",
    "pack_teensy",
    "unpack_sim",
    "unpack_teensy",
]
