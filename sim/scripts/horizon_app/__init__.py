"""HORIZON — Apex Ground (Space Raiders / Raider Aerospace Society).

Ground software for the Apex flight computer: Sensors (USB serial), Radio
(RTL-SDR 2-GFSK downlink), HIL (closed-loop sim), and Logs (device-file
pipeline). See main_window.py for assembly; the line protocol and worker I/O are
the firmware-integration contract and are unchanged from the original monolith.
"""

from .main_window import MainWindow, main

__all__ = ["MainWindow", "main"]
