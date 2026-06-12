"""Serial link to a Teensy running the APEX_HIL firmware build.

``HilLink`` owns the serial port and a background receive thread that
separates the two interleaved streams coming back from the Teensy:

* binary ``TeensyPacket`` frames (synced on the 0xAB 0xCD magic pair), and
* ASCII status lines (``#LEVEL: msg``, ``>key:value``, ``!key:value``).

The main thread drives the loop with :meth:`transact` — send one SimPacket,
block until the matching reply arrives (or a timeout expires).
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional

import serial

from apex_sim.hil.protocol import (
    HIL_BAUD,
    HIL_MAGIC_TEENSY_TO_SIM,
    TEENSY_SIZE,
    SimSensors,
    TeensyPkt,
    pack_sim,
    unpack_teensy,
)


class HilLink:
    """Bidirectional HIL serial link with a background packet/line parser.

    Parameters
    ----------
    port : str
        Serial device path (e.g. ``/dev/cu.usbmodem12345``, or a pty slave
        when talking to the in-process fake Teensy).
    timeout : float
        Per-read timeout for the receive thread in seconds.
    """

    _MAGIC_B0 = HIL_MAGIC_TEENSY_TO_SIM & 0xFF          # 0xAB
    _MAGIC_B1 = (HIL_MAGIC_TEENSY_TO_SIM >> 8) & 0xFF   # 0xCD

    def __init__(self, port: str, timeout: float = 0.05):
        try:
            self._ser = serial.Serial(port, HIL_BAUD, timeout=timeout)
        except OSError:
            # macOS ptys (FakeTeensy) reject the IOSSIOSPEED ioctl for
            # non-standard baud rates. Rate is meaningless on both a pty and
            # the Teensy's USB CDC, so any standard value works.
            self._ser = serial.Serial(port, 115200, timeout=timeout)
        self._cond = threading.Condition()
        self._pkts: List[TeensyPkt] = []
        self._lines: List[str] = []
        self._lines_read = 0
        self._run = True
        self.rx_crc_errors = 0
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def close(self) -> None:
        """Stop the receive thread and close the port."""
        self._run = False
        try:
            self._ser.cancel_read()
        except Exception:
            pass
        self._thread.join(timeout=1.0)
        self._ser.close()

    def reset_input(self) -> None:
        """Flush any stale bytes (e.g. boot banner from a previous session)."""
        self._ser.reset_input_buffer()

    # ── TX ────────────────────────────────────────────────────────────────────

    def send(self, sim_time_ms: int, sensors: SimSensors) -> None:
        """Send one SimPacket without waiting for a reply (warm-up phase)."""
        self._ser.write(pack_sim(sim_time_ms, sensors))

    def transact(self, sim_time_ms: int, sensors: SimSensors,
                 timeout: float = 0.05) -> Optional[TeensyPkt]:
        """Send one SimPacket and wait for the next TeensyPacket reply.

        Returns the reply, or None if nothing valid arrived within
        ``timeout`` seconds (the caller should hold its previous command).
        """
        with self._cond:
            n_before = len(self._pkts)
        self._ser.write(pack_sim(sim_time_ms, sensors))
        deadline = time.monotonic() + timeout
        with self._cond:
            while len(self._pkts) <= n_before:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)
            return self._pkts[-1]

    # ── RX accessors ──────────────────────────────────────────────────────────

    def packets(self) -> List[TeensyPkt]:
        """All TeensyPackets received so far."""
        with self._cond:
            return list(self._pkts)

    def drain_lines(self) -> List[str]:
        """ASCII lines received since the last call."""
        with self._cond:
            new = self._lines[self._lines_read:]
            self._lines_read = len(self._lines)
            return new

    def wait_for_line(self, needle: str, timeout: float) -> bool:
        """Block until a received line contains ``needle``. False on timeout."""
        deadline = time.monotonic() + timeout
        scanned = 0
        while time.monotonic() < deadline:
            with self._cond:
                lines = self._lines[scanned:]
                scanned = len(self._lines)
            if any(needle in ln for ln in lines):
                return True
            time.sleep(0.02)
        return False

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        buf = bytearray()
        synced = False
        while self._run:
            try:
                raw = self._ser.read(1)
            except Exception:
                break
            if not raw:
                continue
            b = raw[0]
            if not synced:
                buf.append(b)
                if len(buf) >= 2 and buf[-2] == self._MAGIC_B0 and buf[-1] == self._MAGIC_B1:
                    synced = True
                    buf = bytearray(buf[-2:])
                elif b == ord("\n") and buf:
                    line = buf.decode("utf-8", errors="replace").rstrip("\r\n")
                    buf.clear()
                    if line:
                        with self._cond:
                            self._lines.append(line)
                            self._cond.notify_all()
                continue
            buf.append(b)
            if len(buf) < TEENSY_SIZE:
                continue
            pkt = unpack_teensy(bytes(buf))
            buf.clear()
            synced = False
            if pkt is not None:
                with self._cond:
                    self._pkts.append(pkt)
                    self._cond.notify_all()
            else:
                self.rx_crc_errors += 1
