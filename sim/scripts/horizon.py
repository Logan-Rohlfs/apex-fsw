#!/usr/bin/env python3
"""
HORIZON — Apex Ground

Ground-software app for the Apex flight computer, named for the HORIZON
tracking groundstation antenna. Four pages, switched in the header tab strip:
Sensors / Radio / HIL / Logs.

Three live data sources:
  USB Serial   — laptop-connected Teensy (teensy41_debug build), full sensor set
  RTL-SDR      — 2-GFSK telemetry downlink (441.480 MHz) with live spectrum,
                 waterfall, ground-station plot layout, and link stats
  HIL Sim      — closed-loop hardware-in-the-loop: runs the RocketPy sim
                 against a Teensy (teensy41_hil build) or the in-process fake
                 flight computer; shows injected sensors, state estimation vs
                 truth, state machine, and airbrake deployment

All sources speak the same line protocol:
  >key:value    numeric — routed to live plots and values table
  !key:value    state   — routed to state panel (phase, health, link, etc.)
  #LEVEL: msg   log     — shown in the log panel with color coding

Commands (USB serial only): ARM, DISARM, TELEM_ON, TELEM_OFF,
RADIO_DATA_TEST, RADIO_MARKER.

The Logs page is a top-to-bottom device-file pipeline:
Flight Computer (files on the device) → Laptop Archive (raw .APXLOG binaries
in sim/output/raw_logs) → CSV Exports (decoded per-flight CSVs).

Run: python scripts/horizon.py
(scripts/monitor.py is a deprecated compatibility shim.)
"""

from __future__ import annotations

import html
import csv
import os
import re
import sys
import time
import shutil
import subprocess
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

# apex_sim package (HIL source) lives one level up from scripts/
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
_SIM_ROOT = Path(__file__).resolve().parents[1]
_RAW_LOG_ARCHIVE = _SIM_ROOT / "output" / "raw_logs"
_FC_LOG_ARCHIVE = _RAW_LOG_ARCHIVE / "flight_computer"
_DELETED_LOG_ARCHIVE = _SIM_ROOT / "output" / "raw_logs_deleted"
_DELETE_CONFIRM_PHRASE = "yes i really do want to delete these files"
_FORMAT_QSPI_CONFIRM_PHRASE = "yes i really do want to format qspi flash"

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSplitter, QScrollArea,
    QFrame, QTextEdit, QGridLayout, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QLineEdit, QCompleter,
    QProxyStyle, QStyle, QTabBar, QFileDialog, QListWidget,
    QListWidgetItem, QAbstractItemView,
    QTableWidget, QTableWidgetItem, QHeaderView, QInputDialog,
    QMessageBox,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF, QPoint, QUrl
from PyQt5.QtGui import QFont, QColor, QPainter, QPolygon, QDesktopServices

from radio_gfsk_rx import (
    BITRATE_BPS,
    DEFAULT_FREQ_HZ,
    DEFAULT_SAMPLE_RATE_HZ,
    DEVIATION_HZ,
    DecodeStats,
    FRAME_TYPE_FLIGHT,
    FRAME_TYPE_HK,
    find_frames,
    load_u8_iq,
)

# Expected flight-computer TX channel (matches fsw RADIO_FREQ_HZ / GFSK params).
# Drawn as an overlay so RADIO_MARKER / RADIO_DATA_TEST energy can be compared
# against where the firmware *should* be transmitting.
EXPECTED_TX_HZ = DEFAULT_FREQ_HZ
EXPECTED_TX_BW_HZ = 2 * (DEVIATION_HZ + BITRATE_BPS // 2)   # Carson bandwidth

# ─── Plot group definitions ───────────────────────────────────────────────────
# (group_title, [keys...], [hex_colors...])
# Keys matched against incoming >key:value lines. Unmatched keys go to "Other".

PLOT_GROUPS = [
    ("Fusion",    ["alt_agl", "velocity", "pred_apogee", "vert_accel"],
                  ["#00d4ff", "#ff6b35", "#a8ff3e", "#bb88ff"]),
    ("Airbrakes", ["deployment", "servo_angle"],            ["#00d4ff", "#ff9933"]),
    ("IMU Accel", ["accel_x", "accel_y", "accel_z"],       ["#ff4444", "#44ff44", "#4488ff"]),
    ("IMU Gyro",  ["gyro_x",  "gyro_y",  "gyro_z"],        ["#ff8844", "#88ff44", "#4488ff"]),
    ("High-G",    ["highg_x", "highg_y", "highg_z"],       ["#ff44aa", "#44ffaa", "#aa44ff"]),
    ("Baro Alt",  ["baro_alt"],                               ["#ffdd44"]),
    ("Baro Temp", ["baro_temp"],                             ["#ff9922"]),
    ("Mag",       ["mag_x",   "mag_y",   "mag_z"],          ["#ff9933", "#33ff99", "#9933ff"]),
]

# Ground-station layout for the RTL-SDR source — keys match the FLIGHT/HK
# telemetry frames (radio_gfsk_rx.py), tuned for flight watching + HORIZON ops.
RADIO_PLOT_GROUPS = [
    ("Altitude",         ["alt_agl", "baro_alt", "pred_apogee", "gps_alt_msl"],
                         ["#00d4ff", "#ffdd44", "#a8ff3e", "#ff44aa"]),
    ("Velocity / Accel", ["velocity", "vert_accel", "accel_z"],
                         ["#ff6b35", "#44ff44", "#4488ff"]),
    ("Roll / Airbrake",  ["roll_rate", "deployment"],
                         ["#ff8844", "#00d4ff"]),
    ("Link",             ["packet_loss_pct", "radio_rx_quality", "radio_freq_offset_khz"],
                         ["#ff4444", "#44ffaa", "#ffdd44"]),
]

# HIL layout — closed-loop sim view. est_* comes from the flight computer's
# TeensyPackets, true_* from the RocketPy state, sim_* is what the emulator
# injects. Dim gray = ground truth "ghost" under the FC's estimate.
HIL_PLOT_GROUPS = [
    ("State Estimation", ["est_alt", "true_alt", "pred_apogee", "fake_est_alt", "fake_pred_apogee"],
                         ["#00d4ff", "#8888aa", "#a8ff3e", "#ff44aa", "#ffaa44"]),
    ("Velocity",         ["est_vel", "true_vel", "fake_est_vel"],
                         ["#ff6b35", "#8888aa", "#ff44aa"]),
    ("Airbrakes",        ["deployment", "fake_deployment"],
                         ["#00d4ff", "#ff44aa"]),
    ("Injected Accel",   ["sim_accel_x", "sim_accel_y", "sim_accel_z"],
                         ["#ff4444", "#44ff44", "#4488ff"]),
    ("Injected Gyro",    ["sim_gyro_x", "sim_gyro_y", "sim_gyro_z"],
                         ["#ff8844", "#88ff44", "#4488ff"]),
    ("Injected Baro",    ["sim_baro_pa"],
                         ["#ffdd44"]),
    ("Loop Latency",     ["hil_lat_ms", "fake_hil_lat_ms"],
                         ["#ff44aa", "#44ffaa"]),
    ("Fake Shadow Delta", ["fake_delta_alt", "fake_delta_vel", "fake_delta_deployment"],
                          ["#ff44aa", "#44ffaa", "#ffaa44"]),
]

HIL_EVENT_STYLES = {
    "launch":          ("Launch", "#44dd88"),
    "burnout":         ("Burnout", "#ff9933"),
    "first_airbrake":  ("First airbrake", "#00d4ff"),
    "apogee":          ("Apogee", "#ff44aa"),
}

# Flight-info reference values — mirror fsw/src/config.h / airbrakes.yaml
TARGET_APOGEE_M = 3048.0    # 10,000 ft AGL
MACH_GATE_MPS   = 240.0     # Mach 0.7 — deployment allowed below this
FT_PER_M        = 3.28084

# App pages (tab index → source mode)
PAGES = [
    ("Sensors", "serial"),
    ("Radio", "radio"),
    ("HIL", "hil"),
    ("Logs", "logs"),
]

# Sensor health bitmask (must match config.h)
SENSOR_BITS = {"IMU": 0, "HG": 1, "BAR": 2, "MAG": 3}

GPS_FIX_LABELS = {-1: "OFFLINE", 0: "SEARCHING", 1: "DR", 2: "2D", 3: "3D", 4: "3D+DR"}

PHASE_COLORS = {
    "IDLE":    "#555566",
    "ARMED":   "#2277dd",
    "BOOST":   "#dd5500",
    "COAST":   "#22aa44",
    "DESCENT": "#ccaa00",
    "LANDED":  "#884499",
    "UNKNOWN": "#333333",
}

MAX_POINTS   = 6000   # rolling buffer depth per series (~60s at 100 Hz)
PLOT_UPDATE_HZ = 25   # UI repaint rate
DEFAULT_WINDOW_S = 30

# ─── Spectrum / waterfall (RTL-SDR) ──────────────────────────────────────────
FFT_BINS       = 1024   # spectrum resolution
SPECTRUM_AVG   = 6      # FFTs averaged per displayed frame
SPECTRUM_FPS   = 30     # spectrum frames emitted per second
WATERFALL_ROWS = 100    # waterfall history depth (rows of FFT_BINS)

# ─── Theme palette ────────────────────────────────────────────────────────────
# One place for every surface/accent color so panels stay cohesive.

ACCENT    = "#00d4ff"   # primary accent — matches the alt_agl trace
BG        = "#0d0d1a"   # window background
SURFACE   = "#13132a"   # raised panels, plot canvas
INSET     = "#09091a"   # input fields, log background
BORDER    = "#2a2a44"
BORDER_HI = "#3a3a66"
TEXT      = "#ccccdd"
TEXT_DIM  = "#8888aa"
GOOD      = "#22aa44"
BAD       = "#aa2222"
AMBER     = "#aa8800"


def badge_style(bg: str, fg: str = "#ffffff", radius: int = 4) -> str:
    return f"background:{bg}; color:{fg}; border-radius:{radius}px; padding:0 6px;"


def mono_font(size: int, bold: bool = False) -> QFont:
    f = QFont("Menlo", size, QFont.Bold if bold else QFont.Normal)
    f.setStyleHint(QFont.Monospace)
    return f


class HorizonStyle(QProxyStyle):
    """Paint small control arrows without depending on glyph fonts or assets."""

    def drawPrimitive(self, element, option, painter, widget=None):
        arrow_elements = {
            QStyle.PE_IndicatorArrowDown,
            QStyle.PE_IndicatorSpinUp,
            QStyle.PE_IndicatorSpinDown,
        }
        if element in arrow_elements and isinstance(widget, (QComboBox, QSpinBox, QDoubleSpinBox)):
            self._draw_arrow(element, option, painter)
            return
        super().drawPrimitive(element, option, painter, widget)

    def _draw_arrow(self, element, option, painter):
        rect = option.rect
        if rect.isEmpty():
            return

        size = max(5, min(9, min(rect.width(), rect.height()) - 4))
        half = size // 2
        center = rect.center()
        color = QColor(TEXT_DIM if option.state & QStyle.State_Enabled else BORDER_HI)

        if element == QStyle.PE_IndicatorSpinUp:
            points = [
                QPoint(center.x(), center.y() - half),
                QPoint(center.x() - half, center.y() + half),
                QPoint(center.x() + half, center.y() + half),
            ]
        else:
            points = [
                QPoint(center.x() - half, center.y() - half),
                QPoint(center.x() + half, center.y() - half),
                QPoint(center.x(), center.y() + half),
            ]

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(Qt.NoPen)
        painter.setBrush(color)
        painter.drawPolygon(QPolygon(points))
        painter.restore()


KNOWN_COMMANDS = [
    "ARM",
    "DISARM",
    "FORMAT_QSPI_ERASE_ALL",   # explicit GUI-confirmed QSPI erase/reformat
    "LIST_LOGS",
    "RADIO_DATA_TEST",
    "RADIO_MARKER",
    "RADIO_MARKER_441",
    "TELEM_ON",
    "TELEM_OFF",
]

# ─── Serial worker ────────────────────────────────────────────────────────────

class SerialWorker(QThread):
    line_received  = pyqtSignal(str)
    status_changed = pyqtSignal(str, bool)   # (message, is_error)
    reconnected    = pyqtSignal()            # Teensy reset detected

    def __init__(self):
        super().__init__()
        self._port     = ""
        self._baud     = 115200
        self._running  = False
        self._ser: serial.Serial | None = None

    def configure(self, port: str, baud: int):
        self._port = port
        self._baud = baud

    def send_bytes(self, data: bytes):
        """Write bytes to the serial port. Thread-safe (pyserial acquires its own lock)."""
        ser = self._ser
        if ser is not None and ser.is_open:
            try:
                ser.write(data)
            except serial.SerialException:
                pass

    def _read_loop(self, ser):
        """Inner read loop. Returns True if we should reconnect, False to exit."""
        buf = ""
        while self._running:
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                return True   # device disconnected — try to reconnect
            if chunk:
                buf += chunk.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.rstrip("\r")
                    if line:
                        self.line_received.emit(line)
        return False

    def run(self):
        self._running = True
        first_connect = True

        while self._running:
            self.status_changed.emit(
                f"{'Connecting' if first_connect else 'Reconnecting'} to {self._port}…", False)
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
                self._ser = ser
                self.status_changed.emit(f"Connected  {self._port} @ {self._baud}", False)
                if not first_connect:
                    self.reconnected.emit()
                first_connect = False

                should_reconnect = self._read_loop(ser)
                self._ser = None
                ser.close()

                if should_reconnect and self._running:
                    self.status_changed.emit(
                        f"Lost connection — waiting for {self._port}…", False)
                    # Wait for the port to reappear (Teensy re-enumerates after reset)
                    while self._running:
                        time.sleep(0.3)
                        ports = [p.device for p in serial.tools.list_ports.comports()]
                        if self._port in ports:
                            break
            except serial.SerialException:
                if self._running:
                    # Port not available yet — keep polling
                    time.sleep(0.5)

        self._ser = None
        self._running = False

    def stop(self):
        self._running = False
        self.wait(3000)


class RadioWorker(QThread):
    line_received  = pyqtSignal(str)
    # Telemetry data lines carry the frame's true reception time (monotonic s),
    # back-computed from its sample position — frames are decoded in 0.5 s
    # batches, and batch-time stamps would collapse 5 frames onto one plot x.
    line_received_at = pyqtSignal(str, float)
    status_changed = pyqtSignal(str, bool)
    spectrum_ready = pyqtSignal(object)   # float32 dB array, FFT_BINS long, DC-centered

    def __init__(self):
        super().__init__()
        self._running = False
        self._proc: subprocess.Popen | None = None
        self._freq_hz = DEFAULT_FREQ_HZ
        self._sample_rate = DEFAULT_SAMPLE_RATE_HZ
        self._gain = "10"
        self._ppm = 0
        self._device = "0"
        self._packet_count = 0
        self._seen_frames: dict[tuple, float] = {}   # (ftype, seq, payload) → last-seen
        self._recent_seqs: deque = deque(maxlen=50)    # for packet-loss %
        self._frame_times: deque = deque(maxlen=20)    # reception times, for rate
        self._last_telem_log_s = 0.0
        self._last_radio_diag_s = 0.0
        self._last_spec_s = 0.0
        self._decode_busy = False
        self._fft_window = np.hanning(FFT_BINS).astype(np.float32)
        # Hann window power normalization (sum of squares)
        self._win_norm = float(np.sum(self._fft_window ** 2))

    def configure(self, freq_hz: int, sample_rate: int, gain: str, ppm: int, device: str):
        self._freq_hz = freq_hz
        self._sample_rate = sample_rate
        self._gain = gain.strip() or "auto"
        self._ppm = ppm
        self._device = device.strip() or "0"

    def run(self):
        rtl_sdr = shutil.which("rtl_sdr")
        if rtl_sdr is None:
            self.status_changed.emit("rtl_sdr not found — install rtl-sdr or check PATH", True)
            return

        cmd = [
            rtl_sdr,
            "-d", self._device,
            "-f", str(self._freq_hz),
            "-s", str(self._sample_rate),
            "-p", str(self._ppm),
            # Small output blocks — rtl_sdr's default is 256 KiB, which at low
            # sample rates means one write every ~0.5 s and a ~2 fps spectrum.
            "-b", "8192",
            "-",
        ]
        if self._gain.lower() != "auto":
            cmd[1:1] = ["-g", self._gain]

        self._running = True
        self._packet_count = 0
        self._seen_frames.clear()
        self._recent_seqs.clear()
        self._frame_times.clear()

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            self.status_changed.emit(f"Failed to start rtl_sdr: {exc}", True)
            self._running = False
            return

        self.status_changed.emit(
            f"RTL-SDR RX {self._freq_hz / 1e6:.3f} MHz @ {self._sample_rate} S/s gain={self._gain}",
            False,
        )
        self.line_received.emit(
            "#INFO: radio rx waiting for 2-GFSK packets "
            "(TELEM_ON telemetry or RADIO_DATA_TEST; RADIO_MARKER is CW spectrum-only)")

        raw = bytearray()
        max_bytes = int(self._sample_rate * 2 * 4)   # unsigned 8-bit IQ, ~4 s rolling window
        min_bytes = int(self._sample_rate * 2 * 1)   # GFSK frames are ~22 ms; 1 s is plenty
        read_size = 65536
        last_decode_s = 0.0

        try:
            while self._running:
                if self._proc.stdout is None:
                    break
                # read1 returns as soon as any data is available — a plain
                # read(n) would block until n bytes arrive, capping the
                # spectrum rate at the pipe's block cadence.
                chunk = self._proc.stdout.read1(read_size)
                if not chunk:
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.05)
                    continue

                raw.extend(chunk)
                if len(raw) > max_bytes:
                    del raw[:len(raw) - max_bytes]

                now = time.monotonic()

                spec_bytes = FFT_BINS * 2 * SPECTRUM_AVG
                if len(raw) >= spec_bytes and now - self._last_spec_s >= 1.0 / SPECTRUM_FPS:
                    self._last_spec_s = now
                    self._emit_spectrum(raw[-spec_bytes:])

                # Decode runs on its own thread — it chews through the whole
                # rolling buffer (~seconds of CPU) and must never stall reads,
                # or the rtl_sdr pipe backs up and the spectrum stutters.
                if (len(raw) >= min_bytes and now - last_decode_s >= 0.25
                        and not self._decode_busy):
                    last_decode_s = now
                    self._decode_busy = True
                    threading.Thread(
                        target=self._decode_job, args=(bytes(raw), now),
                        daemon=True,
                    ).start()
        finally:
            self._terminate_proc()
            self._running = False

    def _decode_job(self, raw: bytes, now_s: float):
        try:
            self._try_decode(raw, now_s)
        finally:
            self._decode_busy = False

    def _emit_spectrum(self, tail: bytearray):
        """Averaged power spectrum of the newest IQ samples (runs in worker thread)."""
        iq = np.frombuffer(tail, dtype=np.uint8).astype(np.float32)
        iq = (iq - 127.5) * (1.0 / 127.5)
        samples = iq[0::2] + 1j * iq[1::2]

        segs = samples[: FFT_BINS * SPECTRUM_AVG].reshape(SPECTRUM_AVG, FFT_BINS)
        spec = np.fft.fft(segs * self._fft_window, axis=1)
        power = np.mean(np.abs(spec) ** 2, axis=0) / self._win_norm
        db = 10.0 * np.log10(power + 1e-12)
        self.spectrum_ready.emit(np.fft.fftshift(db).astype(np.float32))

    def _try_decode(self, raw: bytes, now_s: float):
        stats = DecodeStats()
        try:
            samples = load_u8_iq(raw)
            frames = find_frames(samples, self._sample_rate, stats=stats)
        except Exception as exc:
            self.line_received.emit(f"#WARN: radio rx decode error: {exc}")
            return

        if not frames and stats.candidates and now_s - self._last_radio_diag_s >= 3.0:
            self._last_radio_diag_s = now_s
            detail = (
                f"#WARN: radio rx saw sync-like energy but no valid packets: "
                f"candidates={stats.candidates} bad_crc={stats.bad_crc} "
                f"unknown_type={stats.unknown_type} best_q={stats.best_quality:.2f}")
            if stats.best_type is not None:
                detail += f" best_type=0x{stats.best_type:02X}"
            if stats.best_crc_rx is not None and stats.best_crc_calc is not None:
                detail += f" crc_rx=0x{stats.best_crc_rx:04X} crc_calc=0x{stats.best_crc_calc:04X}"
            self.line_received.emit(detail)

        # The rolling buffer re-decodes each frame for several seconds —
        # report each (seq, payload) once, then age the dedupe entries out.
        for key, seen_s in list(self._seen_frames.items()):
            if now_s - seen_s > 30.0:
                del self._seen_frames[key]

        n_samples = len(samples)
        for fr in frames:
            key = (fr.ftype, fr.seq, fr.payload)
            if key in self._seen_frames:
                self._seen_frames[key] = now_s
                continue
            self._seen_frames[key] = now_s
            self._packet_count += 1

            # True reception time of this frame (monotonic): the buffer
            # snapshot was taken at now_s and ends "now".
            t_frame = now_s - (n_samples - fr.sample_index) / self._sample_rate

            if fr.ftype in (FRAME_TYPE_FLIGHT, FRAME_TYPE_HK):
                self._emit_telemetry(fr, now_s, t_frame)
            else:
                try:
                    text = fr.payload.decode("ascii")
                except UnicodeDecodeError:
                    text = fr.payload.hex(" ")
                self.line_received.emit(
                    f"#INFO: radio test frame seq={fr.seq} payload=\"{text}\" crc=OK "
                    f"quality={fr.quality:.2f} offset={fr.freq_offset_hz / 1e3:+.2f} kHz"
                )

            self.line_received_at.emit(f">radio_packet_count:{self._packet_count}", t_frame)
            self.line_received_at.emit(f">radio_rx_quality:{fr.quality:.3f}", t_frame)
            self.line_received_at.emit(f">radio_freq_offset_khz:{fr.freq_offset_hz / 1e3:.3f}", t_frame)
            self.line_received.emit("!radio_rx:1")

    def _track_loss(self, seq: int) -> float:
        """Packet-loss % over the last ~50 frames, from seq-number gaps."""
        if self._recent_seqs and abs(seq - self._recent_seqs[-1]) > 1000:
            self._recent_seqs.clear()   # counter reset (reboot / u16 wrap)
        self._recent_seqs.append(seq)
        if len(self._recent_seqs) < 2:
            return 0.0
        span = max(self._recent_seqs) - min(self._recent_seqs) + 1
        return max(0.0, 100.0 * (1.0 - len(set(self._recent_seqs)) / span))

    def _emit_telemetry(self, fr, now_s: float, t_frame: float):
        """Route a FLIGHT/HK frame into the plot/state pipeline, stamped with
        its true reception time so plot points land where the frame arrived."""
        t = fr.parse()
        emit = lambda line: self.line_received_at.emit(line, t_frame)
        loss = self._track_loss(t['seq'])
        emit(f">packet_loss_pct:{loss:.1f}")

        # Link panel stats — measured packet rate from true reception times
        self._frame_times.append(t_frame)
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            rate = (len(self._frame_times) - 1) / span if span > 0 else 0.0
        else:
            rate = 0.0
        emit(f"!rx_seq:{t['seq']}")
        emit(f"!rx_rate:{rate:.1f} Hz")
        emit(f"!rx_loss:{loss:.1f} %")
        emit(f"!rx_quality:{fr.quality:.2f}")
        emit(f"!rx_offset:{fr.freq_offset_hz / 1e3:+.2f} kHz")
        emit(f"!rx_count:{self._packet_count}")

        if fr.ftype == FRAME_TYPE_HK:
            for axis in "xyz":
                emit(f">mag_{axis}:{t[f'mag_{axis}_gauss']:.4f}")
                emit(f">highg_{axis}:{t[f'highg_{axis}_mss']:.2f}")
            emit(f">gyro_x:{t['gyro_x_rads']:.4f}")
            emit(f">gyro_y:{t['gyro_y_rads']:.4f}")
            return

        # FLIGHT frame — same keys the USB serial stream uses, plus extras
        baro_alt = 44330.0 * (1.0 - (max(t['baro_pa'], 1.0) / 101325.0) ** (1.0 / 5.255))
        emit(f">alt_agl:{t['alt_agl_m']:.2f}")
        emit(f">baro_alt:{baro_alt:.1f}")
        emit(f">baro_pa:{t['baro_pa']:.0f}")
        emit(f">baro_temp:{t['baro_temp_c']:.1f}")
        emit(f">velocity:{t['velocity_mps']:.3f}")
        emit(f">pred_apogee:{t['pred_apogee_m']:.1f}")
        emit(f">vert_accel:{t['vert_accel_mps2']:.3f}")
        emit(f">accel_z:{t['accel_z_mss']:.3f}")
        emit(f">roll_rate:{t['roll_rate_rads']:.4f}")
        emit(f">deployment:{t['deployment_frac']:.3f}")
        emit(f">gps_alt_msl:{t['gps_alt_msl_m']:.1f}")
        emit(f">gps_lat:{t['gps_lat_deg']:.6f}")
        emit(f">gps_lon:{t['gps_lon_deg']:.6f}")
        emit(f"!phase:{t['phase_name']}")
        emit(f"!health:{t['health']}")
        emit(f"!airbrakes_authorized:{int(t['airbrakes_authorized'])}")
        emit(f"!servo_powered:{int(t['servo_powered'])}")
        emit(f"!arm_switches_closed:{int(t['arm_switches_closed'])}")
        emit(f"!logging_ready:{int(t['logging_ready'])}")
        emit(f"!gps_time_valid:{int(t['gps_time_valid'])}")
        emit(f"!gps_healthy:{int(t['gps_healthy'])}")
        emit(f"!radio_healthy:{int(t['radio_healthy'])}")
        emit(f"!qspi_healthy:{int(t['qspi_healthy'])}")
        emit(f"!sd_healthy:{int(t['sd_healthy'])}")
        emit(f"!gps_fix:{t['gps_fix']}")
        emit(f"!gps_sats:{t['gps_sats']}")
        emit("!radio:0")   # receiving telemetry proves the TX radio is alive

        # Log line rate-limited — telemetry arrives continuously
        if now_s - self._last_telem_log_s >= 5.0:
            self._last_telem_log_s = now_s
            emit(
                f"#INFO: telemetry {t['callsign']} seq={t['seq']} phase={t['phase_name']} "
                f"alt={t['alt_agl_m']:.1f}m gps={t['gps_sats']}sats "
                f"quality={fr.quality:.2f} offset={fr.freq_offset_hz / 1e3:+.2f} kHz"
            )

    def _terminate_proc(self):
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)

    def stop(self):
        self._running = False
        self._terminate_proc()
        self.wait(3000)


# ─── HIL worker ───────────────────────────────────────────────────────────────

class _HilAborted(Exception):
    """Raised from the tick callback to unwind RocketPy when the user stops."""


class HilWorker(QThread):
    """Runs the closed-loop HIL session (apex_sim.hil.runner) in a thread.

    Every controller tick is translated into the app's native line
    protocol and emitted as one batch, so the existing routing, plots, state
    panel, and log handle HIL data exactly like the other sources. The
    callback runs inside the 100 Hz control loop — it only formats strings
    and emits one queued signal.
    """

    lines_received = pyqtSignal(list)        # batch of protocol lines per tick
    status_changed = pyqtSignal(str, bool)   # (message, is_error)

    def __init__(self):
        super().__init__()
        self._running = False
        self._port = ""
        self._fake = True
        self._speed = 1.0
        self._noise = True
        self._record_csv = True
        self._compare_fake = False
        self._sensor_seed = None
        self._sensor_delay_ms = 0.0
        self._full_flight = False
        self._pad_time = 6.0
        self._post_landed_time = 5.0
        self._tick_count = 0
        self._last_gps_fix = 3   # receiver model starts locked on the pad
        self._first_deployment_seen = False

    def configure(self, port: str, fake: bool, speed: float, noise: bool = True,
                  record_csv: bool = True, compare_fake: bool = False,
                  sensor_seed=None, sensor_delay_ms: float = 0.0,
                  full_flight: bool = False, pad_time: float = 6.0,
                  post_landed_time: float = 5.0):
        self._port = port
        self._fake = fake
        self._speed = speed
        self._noise = noise
        self._record_csv = record_csv
        self._compare_fake = compare_fake and not fake
        self._sensor_seed = sensor_seed
        self._sensor_delay_ms = sensor_delay_ms
        self._full_flight = full_flight
        self._pad_time = pad_time
        self._post_landed_time = post_landed_time

    def stop(self):
        self._running = False
        self.wait(5000)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _csv_header(self, compare_fake: bool):
        header = [
            "t_s", "sim_ms", "true_alt_agl_m", "true_vel_z_mps",
            "sim_accel_x_mss", "sim_accel_y_mss", "sim_accel_z_mss",
            "sim_gyro_x_rads", "sim_gyro_y_rads", "sim_gyro_z_rads",
            "sim_baro_pa",
            "est_alt_agl_m", "est_vel_mps", "pred_apogee_m",
            "deployment_frac", "phase", "latency_ms",
        ]
        if compare_fake:
            header.extend([
                "fake_est_alt_agl_m", "fake_est_vel_mps", "fake_pred_apogee_m",
                "fake_deployment_frac", "fake_phase", "fake_latency_ms",
                "fake_minus_primary_alt_m", "fake_minus_primary_vel_mps",
                "fake_minus_primary_deployment",
            ])
        return header

    def _write_csv_row(self, writer, row, compare_fake: bool):
        from apex_sim.hil.protocol import PHASE_NAMES

        if writer is None:
            return
        reply = row.reply
        out = [
            f"{row.t_s:.3f}",
            str(row.sim_ms),
            f"{row.true_alt_agl_m:.2f}",
            f"{row.true_vel_z_mps:.3f}",
            f"{row.sensors.accel_x_mss:.3f}",
            f"{row.sensors.accel_y_mss:.3f}",
            f"{row.sensors.accel_z_mss:.3f}",
            f"{row.sensors.gyro_x_rads:.4f}",
            f"{row.sensors.gyro_y_rads:.4f}",
            f"{row.sensors.gyro_z_rads:.4f}",
            f"{row.sensors.baro_pa:.1f}",
        ]
        if reply is None:
            out.extend(["", "", "", "", "MISS", f"{row.latency_ms:.2f}"])
        else:
            out.extend([
                f"{reply.est_alt_agl_m:.2f}",
                f"{reply.est_vel_mps:.3f}",
                f"{reply.pred_apogee_m:.1f}",
                f"{reply.deployment_frac:.4f}",
                PHASE_NAMES.get(reply.phase, str(reply.phase)),
                f"{row.latency_ms:.2f}",
            ])

        if compare_fake:
            fake_pkt = row.shadow_replies.get("fake")
            if fake_pkt is None:
                out.extend([""] * 9)
            else:
                out.extend([
                    f"{fake_pkt.est_alt_agl_m:.2f}",
                    f"{fake_pkt.est_vel_mps:.3f}",
                    f"{fake_pkt.pred_apogee_m:.1f}",
                    f"{fake_pkt.deployment_frac:.4f}",
                    PHASE_NAMES.get(fake_pkt.phase, str(fake_pkt.phase)),
                    f"{row.shadow_latencies_ms.get('fake', 0.0):.2f}",
                ])
                if reply is None:
                    out.extend(["", "", ""])
                else:
                    out.extend([
                        f"{fake_pkt.est_alt_agl_m - reply.est_alt_agl_m:.2f}",
                        f"{fake_pkt.est_vel_mps - reply.est_vel_mps:.3f}",
                        f"{fake_pkt.deployment_frac - reply.deployment_frac:.4f}",
                    ])
        writer.writerow(out)

    def _emit_tick(self, row, link, last_phase):
        from apex_sim.hil.protocol import PHASE_NAMES

        self._tick_count += 1
        lines = [
            f">true_alt:{row.true_alt_agl_m:.2f}",
            f">true_vel:{row.true_vel_z_mps:.3f}",
            f">sim_accel_x:{row.sensors.accel_x_mss:.3f}",
            f">sim_accel_y:{row.sensors.accel_y_mss:.3f}",
            f">sim_accel_z:{row.sensors.accel_z_mss:.3f}",
            f">sim_gyro_x:{row.sensors.gyro_x_rads:.4f}",
            f">sim_gyro_y:{row.sensors.gyro_y_rads:.4f}",
            f">sim_gyro_z:{row.sensors.gyro_z_rads:.4f}",
            f">sim_baro_pa:{row.sensors.baro_pa:.1f}",
            f">hil_lat_ms:{row.latency_ms:.2f}",
            f"!hil_ticks:{self._tick_count}",
            f"!hil_lat:{row.latency_ms:.1f} ms",
        ]
        if row.reply is not None:
            lines += [
                f">est_alt:{row.reply.est_alt_agl_m:.2f}",
                f">est_vel:{row.reply.est_vel_mps:.3f}",
                f">pred_apogee:{row.reply.pred_apogee_m:.1f}",
                f">deployment:{row.reply.deployment_frac:.4f}",
            ]
            phase = PHASE_NAMES.get(row.reply.phase, "?")
            if phase != last_phase[0]:
                last_phase[0] = phase
                lines.append(f"!phase:{phase}")
                event = {
                    "BOOST": "launch",
                    "COAST": "burnout",
                    "DESCENT": "apogee",
                }.get(phase)
                if event is not None:
                    lines.append(f"!hil_event:{event}")
            if (not self._first_deployment_seen
                    and row.reply.deployment_frac > 0.005):
                self._first_deployment_seen = True
                lines.append("!hil_event:first_airbrake")
        # GPS fix state from the receiver model (drops >4 g, reacquires) —
        # emitted on change so the state-panel badge tracks boost blackouts.
        gps_fix = 3 if row.sensors.gps_valid else 0
        if gps_fix != self._last_gps_fix:
            self._last_gps_fix = gps_fix
            lines.append(f"!gps_fix:{gps_fix}")
            lines.append(f"#INFO: GPS {'fix re-acquired' if gps_fix else 'fix lost (dynamics > 4 g)'}")
        fake_pkt = row.shadow_replies.get("fake")
        if fake_pkt is not None:
            lines += [
                f">fake_est_alt:{fake_pkt.est_alt_agl_m:.2f}",
                f">fake_est_vel:{fake_pkt.est_vel_mps:.3f}",
                f">fake_pred_apogee:{fake_pkt.pred_apogee_m:.1f}",
                f">fake_deployment:{fake_pkt.deployment_frac:.4f}",
                f">fake_hil_lat_ms:{row.shadow_latencies_ms.get('fake', 0.0):.2f}",
            ]
            if row.reply is not None:
                d_alt = fake_pkt.est_alt_agl_m - row.reply.est_alt_agl_m
                d_vel = fake_pkt.est_vel_mps - row.reply.est_vel_mps
                d_dep = fake_pkt.deployment_frac - row.reply.deployment_frac
                lines += [
                    f">fake_delta_alt:{d_alt:.2f}",
                    f">fake_delta_vel:{d_vel:.3f}",
                    f">fake_delta_deployment:{d_dep:.4f}",
                    f"!hil_shadow:dAlt {d_alt:+.1f} m  dVel {d_vel:+.2f} m/s  dDep {d_dep * 100:+.1f}%",
                ]
        # Forward flight-computer ASCII (#INFO transitions, warnings) live.
        lines.extend(link.drain_lines())
        self.lines_received.emit(lines)

    # ── session ───────────────────────────────────────────────────────────────

    def run(self):
        self._first_deployment_seen = False
        self._running = True
        self._tick_count = 0
        self._last_gps_fix = 3
        fake = None
        fake_shadow = None
        link = None
        shadow_links = {}
        csv_file = None
        csv_writer = None
        out_path = None
        try:
            self.status_changed.emit("HIL: building RocketPy sim…", False)
            import yaml
            from apex_sim.config.loader import load_environment
            from apex_sim.hil.emulator import SensorErrors
            from apex_sim.hil.link import HilLink
            from apex_sim.hil.runner import run_closed_loop
            from apex_sim.sim.environment import build_environment
            from apex_sim.sim.rocket import build_rocket

            env_cfg = load_environment()
            env_cfg.atmosphere.model_override = "standard_atmosphere"
            env = build_environment(env_cfg)
            rocket = build_rocket()
            with (Path(__file__).resolve().parents[1]
                  / "config" / "airbrakes.yaml").open() as fh:
                airbrakes_cfg = yaml.safe_load(fh)
            if not self._running:
                raise _HilAborted()

            if self._record_csv:
                out_dir = _SIM_ROOT / "output"
                out_dir.mkdir(parents=True, exist_ok=True)
                out_path = out_dir / f"hil_gui_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                csv_file = out_path.open("w", newline="")
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(self._csv_header(self._compare_fake))
                self.lines_received.emit([f"#INFO: HIL CSV recording -> {out_path}"])
            else:
                self.lines_received.emit(["#INFO: HIL CSV recording disabled"])

            if self._fake:
                from apex_sim.hil.fake_teensy import FakeTeensy
                fake = FakeTeensy()
                port = fake.port
            else:
                port = self._port
            link = HilLink(port)
            if not self._fake:
                link.reset_input()
            self.status_changed.emit(f"HIL: waiting for flight computer on {port}…", False)
            if not link.wait_for_line("READY", timeout=15.0):
                self.status_changed.emit(
                    "HIL: no #HIL_READY — flash the teensy41_hil build", True)
                return
            self.lines_received.emit(["#INFO: HIL flight computer ready"])

            if self._compare_fake:
                from apex_sim.hil.fake_teensy import FakeTeensy
                fake_shadow = FakeTeensy()
                fake_link = HilLink(fake_shadow.port)
                if not fake_link.wait_for_line("READY", timeout=5.0):
                    fake_link.close()
                    self.status_changed.emit("HIL: fake shadow did not become ready", True)
                    return
                shadow_links["fake"] = fake_link
                self.lines_received.emit([
                    "#INFO: HIL fake-shadow comparison enabled (real hardware primary)",
                    "!hil_shadow:waiting for paired packets",
                ])
            else:
                self.lines_received.emit(["#INFO: HIL fake-shadow comparison disabled"])

            self.status_changed.emit("HIL: warming up on pad…", False)
            last_phase = ["ARMED"]
            sensor_kwargs = {
                "seed": self._sensor_seed,
                "errors": SensorErrors(
                    delay_ticks=max(0, int(round(self._sensor_delay_ms / 10.0)))),
            }

            def tick(row):
                if not self._running:
                    raise _HilAborted()
                self._write_csv_row(csv_writer, row, self._compare_fake)
                self._emit_tick(row, link, last_phase)

            # Full flight: fly through descent + landing (needs a long cap so
            # the main chute reaches the ground). Otherwise stop at apogee.
            result = run_closed_loop(
                link, env, rocket, env_cfg, airbrakes_cfg,
                speed=self._speed, warmup_s=self._pad_time,
                terminate_on_apogee=not self._full_flight,
                max_time=600.0 if self._full_flight else 120.0,
                post_landed_s=self._post_landed_time,
                noise=self._noise,
                sensor_kwargs=sensor_kwargs,
                shadow_links=shadow_links,
                tick_cb=tick)

            apogee_ft = (result.flight.apogee - env.elevation) * 3.28084
            replies = [r.reply for r in result.rows if r.reply is not None]
            max_dep = max((r.deployment_frac for r in replies), default=0.0)
            summary = [
                f"#INFO: HIL flight complete — apogee {apogee_ft:.0f} ft AGL, "
                f"max deployment {max_dep * 100:.0f}%",
                f"#INFO: {len(result.rows)} ticks, {result.missed} missed, "
                f"{result.crc_errors} CRC errors",
                f"!hil_ticks:{len(result.rows)}",
                f"!hil_missed:{result.missed}",
                f"!hil_crc:{result.crc_errors}",
            ]
            if self._compare_fake:
                pairs = [(r.reply, r.shadow_replies.get("fake")) for r in result.rows
                         if r.reply is not None and r.shadow_replies.get("fake") is not None]
                if pairs:
                    max_alt_delta = max(abs(f.est_alt_agl_m - p.est_alt_agl_m)
                                        for p, f in pairs)
                    max_vel_delta = max(abs(f.est_vel_mps - p.est_vel_mps)
                                        for p, f in pairs)
                    max_dep_delta = max(abs(f.deployment_frac - p.deployment_frac)
                                        for p, f in pairs)
                    phase_mismatch = sum(1 for p, f in pairs if p.phase != f.phase)
                    summary.extend([
                        f"#INFO: fake shadow delta max |alt|={max_alt_delta:.1f} m, "
                        f"|vel|={max_vel_delta:.2f} m/s, "
                        f"|deploy|={max_dep_delta * 100:.1f}%, "
                        f"phase mismatches={phase_mismatch}/{len(pairs)}",
                        f"!hil_shadow:max dAlt {max_alt_delta:.1f} m  "
                        f"dVel {max_vel_delta:.2f} m/s  dDep {max_dep_delta * 100:.1f}%",
                    ])
                else:
                    summary.append("#WARN: fake shadow comparison produced no paired replies")
            if out_path is not None:
                summary.append(f"#INFO: HIL CSV saved -> {out_path}")
            self.lines_received.emit(summary)
            self.status_changed.emit("HIL: flight complete", False)
        except _HilAborted:
            self.status_changed.emit("HIL: stopped", False)
        except Exception as exc:  # noqa: BLE001 — surface anything in the UI
            self.status_changed.emit(f"HIL error: {exc}", True)
        finally:
            if csv_file is not None:
                csv_file.close()
            if link is not None:
                link.close()
            for shadow in shadow_links.values():
                shadow.close()
            if fake is not None:
                fake.close()
            if fake_shadow is not None:
                fake_shadow.close()
            self._running = False


# ─── Log operations worker ────────────────────────────────────────────────────

class LogOpsWorker(QThread):
    """Runs one log pull/decode/export job off the UI thread.

    MTP pulls can block for minutes (mtp-getfile allows 300 s per file) and
    decoding large logs takes seconds — neither may stall the UI. One job at
    a time; the Logs page disables its action buttons while a job runs
    (spawn-and-skip, same idiom as the SDR decode path).
    """

    progress = pyqtSignal(str)
    done = pyqtSignal(object)      # job return value
    failed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._job = None

    def start_job(self, job) -> bool:
        """job(progress_cb) -> result. Returns False if one is running."""
        if self.isRunning():
            return False
        self._job = job
        self.start()
        return True

    def run(self):
        try:
            self.done.emit(self._job(self.progress.emit))
        except Exception as exc:  # noqa: BLE001 — surfaced in the UI
            self.failed.emit(str(exc))


def flight_summaries(records) -> list:
    """Per-flight rows for the Logs table from decoded LogRecords.

    Returns [(label, boot_id, n_records, start_utc, duration_s, max_alt_m,
    n_events)], flights first, ground groups last.
    """
    groups: dict = {}
    for rec in records:
        groups.setdefault((rec.flight_id, rec.boot_id), []).append(rec)

    rows = []
    for (fid, bid), recs in sorted(groups.items(),
                                   key=lambda kv: (kv[0][0] == 0, kv[0])):
        times = [r.time_ms for r in recs]
        max_alt = None
        start_utc = ""
        n_events = 0
        for r in recs:
            p = r.payload or {}
            alt = p.get("alt_m")
            if alt is not None and (max_alt is None or alt > max_alt):
                max_alt = alt
            if not start_utc and p.get("utc"):
                start_utc = str(p["utc"])
            if r.record_type == 2:   # LOG_REC_EVENT
                n_events += 1
        label = f"Flight {fid}" if fid else "Ground"
        duration_s = (max(times) - min(times)) / 1000.0 if times else 0.0
        rows.append((label, bid, len(recs), start_utc, duration_s,
                     max_alt, n_events))
    return rows


# ─── Plot widget — passes wheel events up so the scroll area scrolls ─────────

class PlotWidget(pg.PlotWidget):
    """PlotWidget that ignores wheel events so the parent QScrollArea can scroll."""
    def wheelEvent(self, ev):
        ev.ignore()


class CommandInput(QLineEdit):
    """Line edit with terminal-style command history and Tab completion."""
    def __init__(self, commands: list[str], parent=None):
        super().__init__(parent)
        self._commands = sorted(commands)
        self._history: list[str] = []
        self._history_index: int | None = None
        self._draft = ""

        completer = QCompleter(self._commands, self)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        self.setCompleter(completer)

    def remember(self, command: str):
        command = command.strip()
        if not command:
            return
        if self._history and self._history[-1] == command:
            self._history_index = None
            return
        self._history.append(command)
        if len(self._history) > 100:
            self._history = self._history[-100:]
        self._history_index = None

    def keyPressEvent(self, ev):
        key = ev.key()

        if key == Qt.Key_Up:
            self._show_history(-1)
            return
        if key == Qt.Key_Down:
            self._show_history(1)
            return
        if key == Qt.Key_Tab:
            self._complete_inline()
            return

        self._history_index = None
        super().keyPressEvent(ev)

    def _show_history(self, step: int):
        if not self._history:
            return

        if self._history_index is None:
            self._draft = self.text()
            self._history_index = len(self._history)

        self._history_index += step
        if self._history_index < 0:
            self._history_index = 0
        if self._history_index > len(self._history):
            self._history_index = len(self._history)

        if self._history_index == len(self._history):
            self.setText(self._draft)
        else:
            self.setText(self._history[self._history_index])
        self.setCursorPosition(len(self.text()))

    def _complete_inline(self):
        prefix = self.text().strip().upper()
        if not prefix:
            return

        matches = [cmd for cmd in self._commands if cmd.startswith(prefix)]
        if len(matches) == 1:
            self.setText(matches[0])
            self.setCursorPosition(len(matches[0]))
        elif len(matches) > 1:
            common = matches[0]
            for match in matches[1:]:
                while not match.startswith(common):
                    common = common[:-1]
            if len(common) > len(prefix):
                self.setText(common)
                self.setCursorPosition(len(common))
            self.completer().setCompletionPrefix(prefix)
            self.completer().complete()


# ─── Spectrum + waterfall panel (RTL-SDR) ────────────────────────────────────

class SpectrumPanel(QGroupBox):
    """SDR++-style live spectrum and scrolling waterfall for the RTL-SDR source.

    Receives DC-centered dB arrays from RadioWorker.spectrum_ready. All FFT
    work happens in the worker thread; this widget only draws.
    """

    def __init__(self):
        super().__init__("Spectrum")
        self._freqs = None          # x-axis in MHz, FFT_BINS long
        self._ema = None            # smoothed spectrum
        self._peak = None           # peak-hold trace
        self._lo = -80.0            # display floor (dB), auto-tracked
        self._hi = -30.0            # display ceiling (dB), auto-tracked
        self._y_lo = None           # last applied y-range, to avoid churn
        self._y_hi = None

        self._wf = np.full((WATERFALL_ROWS, FFT_BINS), -120.0, dtype=np.float32)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        # Controls row
        ctrl_row = QHBoxLayout()
        ctrl_row.setSpacing(8)
        self.peak_check = QCheckBox("Peak hold")
        self.peak_check.toggled.connect(self._on_peak_toggled)
        ctrl_row.addWidget(self.peak_check)
        self.range_label = QLabel("")
        self.range_label.setFont(mono_font(8))
        self.range_label.setStyleSheet(f"color:{TEXT_DIM};")
        ctrl_row.addWidget(self.range_label)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        # Spectrum plot
        self.spec_plot = PlotWidget(background=SURFACE)
        self.spec_plot.setFixedHeight(170)
        self.spec_plot.showGrid(x=True, y=True, alpha=0.25)
        self.spec_plot.setMouseEnabled(x=False, y=False)
        self.spec_plot.hideButtons()
        for side in ("bottom", "left"):
            axis = self.spec_plot.getAxis(side)
            axis.setPen(pg.mkPen(BORDER_HI))
            axis.setTextPen(pg.mkPen(TEXT_DIM))
        self.spec_plot.getAxis("left").setLabel("dBFS")
        self.spec_curve = self.spec_plot.plot(pen=pg.mkPen(ACCENT, width=1.2))
        self.peak_curve = self.spec_plot.plot(pen=pg.mkPen("#ff6b35", width=1.0))
        layout.addWidget(self.spec_plot)

        # Waterfall
        self.wf_plot = PlotWidget(background=SURFACE)
        self.wf_plot.setFixedHeight(220)
        self.wf_plot.setMouseEnabled(x=False, y=False)
        self.wf_plot.hideButtons()
        self.wf_plot.hideAxis("left")
        self.wf_plot.invertY(True)   # newest row at top, history flows downward
        axis = self.wf_plot.getAxis("bottom")
        axis.setPen(pg.mkPen(BORDER_HI))
        axis.setTextPen(pg.mkPen(TEXT_DIM))
        axis.setLabel("MHz")
        self.wf_plot.setXLink(self.spec_plot)

        self.wf_img = pg.ImageItem(axisOrder="row-major")
        self.wf_img.setLookupTable(
            pg.colormap.get("inferno").getLookupTable(nPts=256))
        self.wf_plot.addItem(self.wf_img)
        layout.addWidget(self.wf_plot)

        # Expected TX channel overlay — shaded band + dashed center line on
        # both plots, so actual RF energy can be compared against where the
        # flight computer should be transmitting.
        tx_c = EXPECTED_TX_HZ / 1e6
        tx_half = EXPECTED_TX_BW_HZ / 2e6
        for plot in (self.spec_plot, self.wf_plot):
            region = pg.LinearRegionItem(
                values=(tx_c - tx_half, tx_c + tx_half), movable=False,
                brush=pg.mkBrush("#00d4ff20"), pen=pg.mkPen("#00d4ff50"))
            region.setZValue(10)
            plot.addItem(region)
            line = pg.InfiniteLine(
                pos=tx_c, angle=90, movable=False,
                pen=pg.mkPen(ACCENT, width=1, style=Qt.DashLine))
            line.setZValue(11)
            plot.addItem(line)

        tx_label = QLabel(
            f"expected TX {tx_c:.3f} MHz ±{EXPECTED_TX_BW_HZ / 2e3:.0f} kHz")
        tx_label.setFont(mono_font(8))
        tx_label.setStyleSheet(f"color:{ACCENT};")
        ctrl_row.insertWidget(ctrl_row.count() - 1, tx_label)

        self.set_params(DEFAULT_FREQ_HZ, DEFAULT_SAMPLE_RATE_HZ)

    def set_params(self, center_hz: int, sample_rate: int):
        """Reset axes/history for a new tune. Call before starting the worker."""
        center = center_hz / 1e6
        span = sample_rate / 1e6
        f_lo = center - span / 2
        self._freqs = np.linspace(f_lo, f_lo + span, FFT_BINS)
        self._ema = None
        self._peak = None
        self._wf.fill(-120.0)
        self.wf_img.setImage(self._wf, autoLevels=False, levels=(self._lo, self._hi))
        self.wf_img.setRect(QRectF(f_lo, 0.0, span, float(WATERFALL_ROWS)))
        self.spec_plot.setXRange(f_lo, f_lo + span, padding=0)
        self.wf_plot.setYRange(0, WATERFALL_ROWS, padding=0)

    def _on_peak_toggled(self, on: bool):
        self._peak = None
        if not on:
            self.peak_curve.setData([], [])

    def update_spectrum(self, db: np.ndarray):
        if not self.isVisible() or self._freqs is None:
            return

        # Smoothed trace
        if self._ema is None:
            self._ema = db.copy()
        else:
            self._ema += 0.35 * (db - self._ema)
        self.spec_curve.setData(self._freqs, self._ema)

        if self.peak_check.isChecked():
            self._peak = db.copy() if self._peak is None else np.maximum(self._peak, db)
            self.peak_curve.setData(self._freqs, self._peak)

        # Waterfall: scroll down one row, newest at top
        self._wf[1:] = self._wf[:-1]
        self._wf[0] = db

        # Auto levels: slow-tracking floor/ceiling from percentiles
        lo = float(np.percentile(db, 10))
        hi = float(np.max(db))
        self._lo += 0.05 * (lo - self._lo)
        self._hi += 0.05 * (hi - self._hi)
        if self._hi - self._lo < 10.0:
            self._hi = self._lo + 10.0

        self.wf_img.setImage(self._wf, autoLevels=False,
                             levels=(self._lo, self._hi + 5.0))

        # Re-range the spectrum y-axis only when it drifts visibly
        y_lo, y_hi = self._lo - 10.0, self._hi + 10.0
        if (self._y_lo is None or abs(y_lo - self._y_lo) > 2.0
                or abs(y_hi - self._y_hi) > 2.0):
            self._y_lo, self._y_hi = y_lo, y_hi
            self.spec_plot.setYRange(y_lo, y_hi, padding=0)
            self.range_label.setText(f"{self._lo:.0f} … {self._hi:.0f} dB")


# ─── Plot group widget ────────────────────────────────────────────────────────

class PlotGroupWidget(QGroupBox):
    def __init__(self, title: str, keys: list, colors: list, window_s: int = DEFAULT_WINDOW_S):
        super().__init__(title)
        self.keys    = keys
        self.colors  = colors

        self._t   = {k: deque(maxlen=MAX_POINTS) for k in keys}
        self._y   = {k: deque(maxlen=MAX_POINTS) for k in keys}
        self._t0  = None   # first timestamp seen
        self._dirty: set[str] = set()      # keys with data since last refresh
        self._last_window = window_s
        self._was_offscreen = False        # skipped while scrolled out / on another page
        self._event_lines = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.plot = PlotWidget(background=SURFACE)
        self.plot.setMinimumHeight(160)
        # Bound the height: raster cost scales with pixel area, so fullscreen
        # should show more plots, not gigantic ones.
        self.plot.setMaximumHeight(260)
        self.plot.setAntialiasing(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        for side in ("bottom", "left"):
            axis = self.plot.getAxis(side)
            axis.setPen(pg.mkPen(BORDER_HI))
            axis.setTextPen(pg.mkPen(TEXT_DIM))
        self.plot.getAxis("bottom").setLabel("t (s)")
        # y auto-ranges; x is scrolled by refresh() against wall-clock time so
        # the view glides at the UI frame rate even when data arrives in bursts
        self.plot.enableAutoRange(axis="y")
        self._last_push = None   # monotonic time of newest sample
        self.plot.addLegend(
            offset=(-10, 10),
            brush=pg.mkBrush("#13132ad0"),
            pen=pg.mkPen(BORDER),
            labelTextColor=TEXT,
        )

        self.curves = {}
        for i, key in enumerate(keys):
            self._make_curve(key, colors[i % len(colors)])

        layout.addWidget(self.plot)

    def _make_curve(self, key: str, color: str):
        curve = self.plot.plot(pen=pg.mkPen(color, width=1.5), name=key,
                               antialias=False)
        curve.setDownsampling(auto=True, method="peak")
        curve.setClipToView(True)   # only render points inside the x-window
        self.curves[key] = curve

    def push(self, key: str, t: float, value: float):
        if key not in self._t:
            return
        if self._t0 is None:
            self._t0 = t
        self._t[key].append(t - self._t0)
        self._y[key].append(value)
        self._dirty.add(key)
        self._last_push = time.monotonic()

    def mark_offscreen(self):
        """Called instead of refresh() while not visible — data keeps
        accumulating via push(); the next refresh() repaints everything."""
        self._was_offscreen = True

    def refresh(self, window_s: int, t_now: float = None):
        reappeared = self._was_offscreen
        self._was_offscreen = False
        if reappeared or window_s != self._last_window:
            self._last_window = window_s
            self._dirty.update(k for k in self.keys if self._t[k])
        if self._dirty:
            for key in self._dirty:
                t_arr = np.array(self._t[key])
                y_arr = np.array(self._y[key])
                i0 = np.searchsorted(t_arr, t_arr[-1] - window_s)
                self.curves[key].setData(t_arr[i0:], y_arr[i0:])
            self._dirty.clear()

        # Smooth scroll: glide the x-window with wall-clock time while data is
        # live; freeze after 5 s of silence so a finished run stays inspectable.
        if (t_now is not None and self._t0 is not None and self._last_push is not None
                and time.monotonic() - self._last_push <= 5.0):
            x_max = t_now - self._t0
            self.plot.setXRange(x_max - window_s, x_max, padding=0)
        elif reappeared and self._t0 is not None:
            # Data went quiet while we were offscreen — re-anchor the frozen
            # view at the newest sample so the run end is what's shown.
            t_latest = max((self._t[k][-1] for k in self.keys if self._t[k]),
                           default=None)
            if t_latest is not None:
                self.plot.setXRange(t_latest - window_s, t_latest, padding=0)

    def clear(self):
        for key in self.keys:
            self._t[key].clear()
            self._y[key].clear()
            self.curves[key].setData([], [])
        self._t0 = None
        self._last_push = None
        self._dirty.clear()
        for line in self._event_lines:
            self.plot.removeItem(line)
        self._event_lines.clear()

    def add_event(self, event_t: float, label: str, color: str):
        """Add a labeled vertical event marker using the group's time axis."""
        if self._t0 is None:
            return
        line = pg.InfiniteLine(
            pos=event_t - self._t0,
            angle=90,
            movable=False,
            pen=pg.mkPen(color, width=1.25, style=Qt.DotLine),
            label=label,
            labelOpts={
                "color": color,
                "position": 0.92,
                "fill": pg.mkBrush("#13132acc"),
                "movable": False,
            },
        )
        line.setZValue(20)
        self.plot.addItem(line)
        self._event_lines.append(line)

    def add_key(self, key: str, color: str = "#ffffff"):
        """Dynamically add a key not in the original definition."""
        if key in self._t:
            return
        self.keys.append(key)
        self.colors.append(color)
        self._t[key] = deque(maxlen=MAX_POINTS)
        self._y[key] = deque(maxlen=MAX_POINTS)
        self._make_curve(key, color)

# ─── Deployment gauge ────────────────────────────────────────────────────────

class DeploymentGauge(QWidget):
    """Horizontal 0–100 % bar for airbrake deployment.

    Driven from the `deployment` numeric key (0.0–1.0), so it works for the
    HIL source and the radio telemetry downlink alike.
    """

    def __init__(self):
        super().__init__()
        self.setFixedHeight(26)
        self._frac = 0.0

    def set_fraction(self, frac: float):
        frac = max(0.0, min(1.0, frac))
        if abs(frac - self._frac) > 0.0005:
            self._frac = frac
            self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        r = self.rect().adjusted(1, 1, -1, -1)

        p.setPen(pg.mkPen(BORDER_HI))
        p.setBrush(QColor(INSET))
        p.drawRoundedRect(r, 4, 4)

        fill = QRectF(r)
        fill.setWidth(r.width() * self._frac)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(ACCENT))
        p.save()
        p.setClipRect(fill)
        p.drawRoundedRect(QRectF(r), 4, 4)
        p.restore()

        p.setFont(mono_font(10, bold=True))
        p.setPen(QColor("#0d0d1a" if self._frac > 0.55 else TEXT))
        p.drawText(r, Qt.AlignCenter, f"{self._frac * 100:.1f}%")
        p.end()


# ─── State panel ─────────────────────────────────────────────────────────────

class StatePanel(QWidget):
    copy_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setFixedWidth(260)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Phase badge
        self.phase_label = QLabel("IDLE")
        self.phase_label.setAlignment(Qt.AlignCenter)
        self.phase_label.setFont(mono_font(22, bold=True))
        self.phase_label.setFixedHeight(54)
        self.phase_label.setStyleSheet(badge_style(PHASE_COLORS["IDLE"], radius=6))
        layout.addWidget(self.phase_label)

        # Flight info — converted/derived values in flight units. Fed from the
        # same numeric keys as the plots (serial/radio: alt_agl, velocity;
        # HIL: est_alt, est_vel), buffered and repainted on the UI tick.
        flight_box = QGroupBox("Flight")
        flight_grid = QGridLayout(flight_box)
        flight_grid.setContentsMargins(6, 4, 6, 4)
        flight_grid.setVerticalSpacing(2)
        self._flight_labels: dict[str, QLabel] = {}

        def flight_row(row: int, title: str, key: str, size: int = 10):
            k = QLabel(title)
            k.setFont(mono_font(9))
            k.setStyleSheet("color:#aaaacc;")
            v = QLabel("—")
            v.setFont(mono_font(size, bold=True))
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            v.setStyleSheet("color:#ffffff;")
            flight_grid.addWidget(k, row, 0)
            flight_grid.addWidget(v, row, 1)
            self._flight_labels[key] = v

        flight_row(0, "Alt",         "f_alt", size=12)
        flight_row(1, "Max alt",     "f_max")
        flight_row(2, "Pred apogee", "f_pred", size=12)
        flight_row(3, "vs 10k ft",   "f_err")
        flight_row(4, "Speed",       "f_speed", size=12)
        flight_row(5, "Deploy gate", "f_gate")
        self._flight_labels["f_gate"].setText(f"< {MACH_GATE_MPS:.0f} m/s")
        self._flight_labels["f_gate"].setStyleSheet(f"color:{TEXT_DIM};")
        layout.addWidget(flight_box)

        self._flight_alt_m = None      # latest values, flushed on UI tick
        self._flight_max_m = 0.0
        self._flight_pred_m = None
        self._flight_speed = None

        # Airbrake deployment gauge — fed by the `deployment` numeric key
        # (HIL replies and radio telemetry both carry it)
        gauge_box = QGroupBox("Airbrakes")
        gauge_layout = QVBoxLayout(gauge_box)
        gauge_layout.setContentsMargins(6, 4, 6, 4)
        self.deploy_gauge = DeploymentGauge()
        gauge_layout.addWidget(self.deploy_gauge)
        layout.addWidget(gauge_box)

        # Sensor health row
        health_box = QGroupBox("Sensors")
        health_layout = QHBoxLayout(health_box)
        health_layout.setContentsMargins(6, 4, 6, 4)
        self.sensor_dots: dict[str, QLabel] = {}
        for name in SENSOR_BITS:
            dot = QLabel(name)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(mono_font(9, bold=True))
            dot.setFixedSize(52, 22)
            dot.setStyleSheet(badge_style("#444", TEXT_DIM))
            self.sensor_dots[name] = dot
            health_layout.addWidget(dot)
        layout.addWidget(health_box)

        # System health occupies the upper nibble of the radio health byte.
        # Keep it separate from physical sensors and hide it outside Radio mode.
        self.system_health_box = QGroupBox("Systems")
        system_health_layout = QHBoxLayout(self.system_health_box)
        system_health_layout.setContentsMargins(6, 4, 6, 4)
        system_health_layout.setSpacing(3)
        self.system_health_dots: dict[str, QLabel] = {}
        for name in ("GPS", "RAD", "QSPI", "SD"):
            dot = QLabel(name)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(mono_font(8, bold=True))
            dot.setFixedSize(52, 22)
            dot.setStyleSheet(badge_style("#444", TEXT_DIM))
            self.system_health_dots[name] = dot
            system_health_layout.addWidget(dot)
        layout.addWidget(self.system_health_box)

        # Operational flags share the upper five bits of phase_status.
        self.radio_ops_box = QGroupBox("Flight Interlocks")
        radio_ops_grid = QGridLayout(self.radio_ops_box)
        radio_ops_grid.setContentsMargins(6, 4, 6, 4)
        radio_ops_grid.setHorizontalSpacing(3)
        radio_ops_grid.setVerticalSpacing(3)
        self.radio_ops_dots: dict[str, QLabel] = {}
        for i, (key, label) in enumerate((
                ("airbrakes_authorized", "BRAKES"),
                ("servo_powered", "SERVO"),
                ("arm_switches_closed", "ARM SW"),
                ("logging_ready", "LOG"),
                ("gps_time_valid", "UTC"))):
            dot = QLabel(label)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(mono_font(8, bold=True))
            dot.setMinimumSize(68, 22)
            dot.setStyleSheet(badge_style("#444", TEXT_DIM))
            self.radio_ops_dots[key] = dot
            radio_ops_grid.addWidget(dot, i // 3, i % 3)
        layout.addWidget(self.radio_ops_box)

        # Radio status
        radio_box = QGroupBox("Radio")
        radio_layout = QHBoxLayout(radio_box)
        radio_layout.setContentsMargins(6, 4, 6, 4)
        self.radio_label = QLabel("OFFLINE")
        self.radio_label.setAlignment(Qt.AlignCenter)
        self.radio_label.setFont(mono_font(9, bold=True))
        self.radio_label.setFixedHeight(20)
        self.radio_label.setStyleSheet(badge_style(BAD, radius=3))
        radio_layout.addWidget(self.radio_label)
        layout.addWidget(radio_box)

        # Link stats — TX side over USB serial, RX side over the SDR
        link_box = QGroupBox("Link")
        link_grid = QGridLayout(link_box)
        link_grid.setContentsMargins(6, 4, 6, 4)
        link_grid.setVerticalSpacing(2)
        self._link_value_labels: dict[str, QLabel] = {}
        self._link_serial_widgets: list[QLabel] = []
        self._link_radio_widgets: list[QLabel] = []
        self._link_hil_widgets: list[QLabel] = []

        def link_row(row: int, title: str, key: str, widgets: list):
            k = QLabel(title)
            k.setFont(mono_font(9))
            k.setStyleSheet("color:#aaaacc;")
            v = QLabel("—")
            v.setFont(mono_font(9, bold=True))
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            v.setStyleSheet("color:#ffffff;")
            link_grid.addWidget(k, row, 0)
            link_grid.addWidget(v, row, 1)
            widgets.extend((k, v))
            self._link_value_labels[key] = v

        # Serial mode: what the flight computer reports about its own TX
        link_row(0, "Beacon",  "telem",      self._link_serial_widgets)
        link_row(1, "TX seq",  "tx_seq",     self._link_serial_widgets)
        link_row(2, "Sent",    "tx_sent",    self._link_serial_widgets)
        link_row(3, "Skipped", "tx_skipped", self._link_serial_widgets)
        # Radio mode: what the SDR receiver measures
        link_row(4, "RX seq",  "rx_seq",     self._link_radio_widgets)
        link_row(5, "Rate",    "rx_rate",    self._link_radio_widgets)
        link_row(6, "Loss",    "rx_loss",    self._link_radio_widgets)
        link_row(7, "Quality", "rx_quality", self._link_radio_widgets)
        link_row(8, "Offset",  "rx_offset",  self._link_radio_widgets)
        link_row(9, "Packets", "rx_count",   self._link_radio_widgets)
        # HIL mode: packet-loop stats and optional fake-shadow summary
        link_row(10, "Ticks",   "hil_ticks",  self._link_hil_widgets)
        link_row(11, "Latency", "hil_lat",    self._link_hil_widgets)
        link_row(12, "Missed",  "hil_missed", self._link_hil_widgets)
        link_row(13, "CRC",     "hil_crc",    self._link_hil_widgets)
        link_row(14, "Shadow",  "hil_shadow", self._link_hil_widgets)

        layout.addWidget(link_box)
        self.set_link_mode("serial")

        # GPS status row
        gps_box = QGroupBox("GPS")
        gps_layout = QHBoxLayout(gps_box)
        gps_layout.setContentsMargins(6, 4, 6, 4)

        self.gps_fix_label = QLabel("NO FIX")
        self.gps_fix_label.setFont(mono_font(9, bold=True))
        self.gps_fix_label.setAlignment(Qt.AlignCenter)
        self.gps_fix_label.setFixedHeight(20)
        self.gps_fix_label.setStyleSheet(badge_style("#444", TEXT_DIM, radius=3))

        self.gps_sats_label = QLabel("0 sats")
        self.gps_sats_label.setFont(mono_font(9))
        self.gps_sats_label.setStyleSheet(f"color:{TEXT_DIM};")

        self.gps_utc_label = QLabel("—")
        self.gps_utc_label.setFont(mono_font(8))
        self.gps_utc_label.setStyleSheet(f"color:{TEXT_DIM};")

        gps_layout.addWidget(self.gps_fix_label)
        gps_layout.addWidget(self.gps_sats_label)
        gps_layout.addWidget(self.gps_utc_label, stretch=1)
        layout.addWidget(gps_box)

        # Values table with copy button in header
        values_box = QGroupBox("Values")
        values_outer = QVBoxLayout(values_box)
        values_outer.setContentsMargins(6, 4, 6, 4)
        values_outer.setSpacing(4)

        # Copy row — button uses the same window duration as the plot window
        copy_row = QHBoxLayout()
        copy_row.setSpacing(4)
        copy_row.addWidget(QLabel("Last"))
        self.copy_window_spin = QSpinBox()
        self.copy_window_spin.setRange(1, 600)
        self.copy_window_spin.setValue(DEFAULT_WINDOW_S)
        self.copy_window_spin.setSuffix(" s")
        self.copy_window_spin.setFixedWidth(66)
        copy_row.addWidget(self.copy_window_spin)
        copy_btn = QPushButton("Copy CSV")
        copy_btn.setFixedWidth(72)
        copy_btn.setToolTip("Copy last N seconds of all data to clipboard as CSV")
        copy_btn.clicked.connect(self.copy_requested)
        copy_row.addWidget(copy_btn)
        values_outer.addLayout(copy_row)

        # Grid for key-value pairs
        self._values_grid = QWidget()
        self._values_layout = QGridLayout(self._values_grid)
        self._values_layout.setContentsMargins(0, 0, 0, 0)
        self._values_layout.setVerticalSpacing(2)
        self._value_rows: dict[str, QLabel] = {}
        self._pending_values: dict[str, float] = {}
        values_outer.addWidget(self._values_grid)
        layout.addWidget(values_box)
        layout.addStretch()

    def set_link_mode(self, mode="serial"):
        """Show TX stats (USB), RX packet stats (SDR), or packet-loop stats (HIL)."""
        if isinstance(mode, bool):
            mode = "radio" if mode else "serial"
        for w in self._link_serial_widgets:
            w.setVisible(mode == "serial")
        for w in self._link_radio_widgets:
            w.setVisible(mode == "radio")
        for w in self._link_hil_widgets:
            w.setVisible(mode == "hil")
        self.system_health_box.setVisible(mode == "radio")
        self.radio_ops_box.setVisible(mode == "radio")

    @staticmethod
    def _set_status_dot(dot: QLabel, ok: bool):
        dot.setStyleSheet(badge_style(GOOD if ok else BAD))

    def update_phase(self, phase: str):
        phase = phase.strip().upper()
        color = PHASE_COLORS.get(phase, PHASE_COLORS["UNKNOWN"])
        self.phase_label.setText(phase)
        self.phase_label.setStyleSheet(badge_style(color, radius=6))

    def update_health(self, bitmask: int):
        for name, bit in SENSOR_BITS.items():
            ok = bool(bitmask & (1 << bit))
            self.sensor_dots[name].setStyleSheet(badge_style(GOOD if ok else BAD))

    def update_value(self, key: str, value: float):
        # Buffered — labels repaint on the UI tick via flush_values(), not at
        # the incoming serial rate.
        self._pending_values[key] = value
        if key in ("alt_agl", "est_alt"):
            self._flight_alt_m = value
            if value > self._flight_max_m:
                self._flight_max_m = value
        elif key == "pred_apogee":
            self._flight_pred_m = value
        elif key in ("velocity", "est_vel"):
            self._flight_speed = value

    def reset_flight(self):
        self._flight_alt_m = None
        self._flight_max_m = 0.0
        self._flight_pred_m = None
        self._flight_speed = None
        for key in ("f_alt", "f_max", "f_pred", "f_err", "f_speed"):
            self._flight_labels[key].setText("—")
            self._flight_labels[key].setStyleSheet("color:#ffffff;")
        self.deploy_gauge.set_fraction(0.0)
        for dot in self.system_health_dots.values():
            dot.setStyleSheet(badge_style("#444", TEXT_DIM))
        for dot in self.radio_ops_dots.values():
            dot.setStyleSheet(badge_style("#444", TEXT_DIM))

    def _flush_flight(self):
        if self._flight_alt_m is not None:
            self._flight_labels["f_alt"].setText(f"{self._flight_alt_m * FT_PER_M:,.0f} ft")
            self._flight_labels["f_max"].setText(f"{self._flight_max_m * FT_PER_M:,.0f} ft")
        if self._flight_pred_m is not None:
            err_ft = (self._flight_pred_m - TARGET_APOGEE_M) * FT_PER_M
            self._flight_labels["f_pred"].setText(
                f"{self._flight_pred_m * FT_PER_M:,.0f} ft")
            err = self._flight_labels["f_err"]
            err.setText(f"{err_ft:+,.0f} ft")
            tol = abs(err_ft)
            err.setStyleSheet("color:%s;" % (
                GOOD if tol < 100 else AMBER if tol < 300 else "#ff6666"))
        if self._flight_speed is not None:
            speed = self._flight_labels["f_speed"]
            speed.setText(f"{self._flight_speed:.1f} m/s")
            # Green = below the mach gate (deployment allowed), amber above
            below = self._flight_speed < MACH_GATE_MPS
            speed.setStyleSheet("color:%s;" % (GOOD if below else AMBER))

    def flush_values(self):
        self._flush_flight()
        if not self._pending_values:
            return
        if "deployment" in self._pending_values:
            self.deploy_gauge.set_fraction(self._pending_values["deployment"])
        for key, value in self._pending_values.items():
            if key not in self._value_rows:
                row = self._values_layout.rowCount()
                key_label = QLabel(key)
                key_label.setFont(mono_font(9))
                key_label.setStyleSheet("color:#aaaacc;")
                val_label = QLabel("—")
                val_label.setFont(mono_font(9, bold=True))
                val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                val_label.setStyleSheet("color:#ffffff;")
                self._values_layout.addWidget(key_label, row, 0)
                self._values_layout.addWidget(val_label, row, 1)
                self._value_rows[key] = val_label
            self._value_rows[key].setText(f"{value:.4g}")
        self._pending_values.clear()

    def update_state(self, key: str, value: str):
        """Handle arbitrary !key:value state lines."""
        if key == "phase":
            self.update_phase(value)
        elif key == "health":
            try:
                self.update_health(int(value))
            except ValueError:
                pass
        elif key in ("gps_healthy", "radio_healthy", "qspi_healthy", "sd_healthy"):
            names = {
                "gps_healthy": "GPS", "radio_healthy": "RAD",
                "qspi_healthy": "QSPI", "sd_healthy": "SD",
            }
            try:
                self._set_status_dot(self.system_health_dots[names[key]], int(value) != 0)
            except ValueError:
                pass
        elif key in self.radio_ops_dots:
            try:
                self._set_status_dot(self.radio_ops_dots[key], int(value) != 0)
            except ValueError:
                pass
        elif key == "gps_fix":
            try:
                fix = int(value)
                label = GPS_FIX_LABELS.get(fix, str(fix))
                self.gps_fix_label.setText(label)
                if fix >= 3:
                    bg = GOOD       # green — 3D fix
                elif fix >= 0:
                    bg = AMBER      # amber — online, searching/2D
                else:
                    bg = BAD        # red — offline / init failed
                self.gps_fix_label.setStyleSheet(badge_style(bg, radius=3))
            except ValueError:
                pass
        elif key == "gps_sats":
            self.gps_sats_label.setText(f"{value} sats")
        elif key == "utc":
            self.gps_utc_label.setText(value)
            self.gps_utc_label.setStyleSheet("color:#aaffaa;")
        elif key == "radio":
            try:
                s = int(value)
                if s >= 0:
                    self.radio_label.setText("Si4463 OK")
                    self.radio_label.setStyleSheet(badge_style(GOOD, radius=3))
                else:
                    self.radio_label.setText("OFFLINE")
                    self.radio_label.setStyleSheet(badge_style(BAD, radius=3))
            except ValueError:
                pass
        elif key == "radio_rx":
            try:
                ok = int(value) >= 0
                self.radio_label.setText("RX OK" if ok else "RX BAD")
                self.radio_label.setStyleSheet(badge_style(GOOD if ok else AMBER, radius=3))
            except ValueError:
                pass
        elif key == "telem":
            lbl = self._link_value_labels["telem"]
            on = value.strip() == "1"
            lbl.setText("ON" if on else "OFF")
            lbl.setStyleSheet(f"color:{GOOD if on else TEXT_DIM};")
        elif key in self._link_value_labels:
            self._link_value_labels[key].setText(value)

# ─── Main window ─────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HORIZON — Apex Ground")
        self.resize(1280, 820)
        self._apply_dark_theme()

        self._worker = SerialWorker()
        self._radio_worker = RadioWorker()
        self._hil_worker = HilWorker()
        self._log_ops = LogOpsWorker()
        self._log_export_after_pull = False
        self._last_export_dir: Path | None = None
        self._t_start = None
        self._window_s = DEFAULT_WINDOW_S

        # Key → PlotGroupWidget mapping for routing, per source
        self._key_to_group_serial: dict[str, PlotGroupWidget] = {}
        self._key_to_group_radio: dict[str, PlotGroupWidget] = {}
        self._key_to_group_hil: dict[str, PlotGroupWidget] = {}
        self._overflow_group: PlotGroupWidget | None = None
        # Device file listing from the last Refresh (plain dicts — safe to
        # hand to the LogOpsWorker job closures)
        self._device_entries: list[dict] = []
        self._visible_device_entries: list[dict] = []
        self._local_log_info_cache: dict[str, tuple[tuple[int, int], dict]] = {}

        # Debounced restart so gain/freq/ppm changes apply while connected
        self._retune_timer = QTimer(self)
        self._retune_timer.setSingleShot(True)
        self._retune_timer.setInterval(500)
        self._retune_timer.timeout.connect(self._apply_radio_retune)

        self._build_ui()
        self._connect_signals()

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_plots)
        self._timer.start(1000 // PLOT_UPDATE_HZ)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # App chrome — the tab strip doubles as the branded header: HORIZON
        # wordmark on the left, page tabs (Sensors / Radio / HIL / Logs)
        # alongside. Each tab swaps the toolbar controls and plot layout;
        # panels themselves are shared.
        tab_strip = QFrame()
        tab_strip.setObjectName("tabstrip")
        tab_strip.setFixedHeight(34)
        tab_layout = QHBoxLayout(tab_strip)
        tab_layout.setContentsMargins(10, 0, 10, 0)
        tab_layout.setSpacing(0)

        wordmark = QLabel("HORIZON")
        wordmark.setFont(mono_font(12, bold=True))
        wordmark.setStyleSheet(f"color:{ACCENT}; background:transparent;")
        tab_layout.addWidget(wordmark)
        wordmark_sub = QLabel("APEX GROUND")
        wordmark_sub.setFont(mono_font(8))
        wordmark_sub.setStyleSheet(f"color:{TEXT_DIM}; background:transparent;")
        tab_layout.addSpacing(8)
        tab_layout.addWidget(wordmark_sub)
        tab_layout.addSpacing(18)

        self.tab_bar = QTabBar()
        self.tab_bar.setExpanding(False)
        self.tab_bar.setDrawBase(False)
        self.tab_bar.setFocusPolicy(Qt.NoFocus)
        for title, _mode in PAGES:
            self.tab_bar.addTab(title)
        self.tab_bar.currentChanged.connect(self._on_source_changed)
        tab_layout.addWidget(self.tab_bar)
        tab_layout.addStretch()
        root.addWidget(tab_strip)

        # Toolbar — fixed-height control bar with bottom separator
        root.addWidget(self._build_toolbar())

        # Main splitter: plots | state+log
        splitter = QSplitter(Qt.Horizontal)
        splitter.setContentsMargins(6, 6, 6, 6)
        root.addWidget(splitter, stretch=1)

        splitter.addWidget(self._build_plot_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setSizes([860, 260])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("toolbar")
        bar.setFixedHeight(42)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(6)

        # Port selector
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(160)
        self._refresh_ports()
        layout.addWidget(self.port_label)
        layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedWidth(28)
        self.refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(self.refresh_btn)

        # Baud selector
        self.baud_label = QLabel("Baud:")
        self.baud_combo = QComboBox()
        for b in ["9600", "57600", "115200", "230400", "460800", "921600"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("921600")
        layout.addWidget(self.baud_label)
        layout.addWidget(self.baud_combo)

        # RTL-SDR controls
        self.radio_freq_label = QLabel("Freq:")
        self.radio_freq_spin = QDoubleSpinBox()
        self.radio_freq_spin.setRange(100.0, 1000.0)
        self.radio_freq_spin.setDecimals(3)
        self.radio_freq_spin.setSingleStep(0.001)
        self.radio_freq_spin.setValue(DEFAULT_FREQ_HZ / 1e6)
        self.radio_freq_spin.setSuffix(" MHz")
        self.radio_freq_spin.setFixedWidth(104)
        layout.addWidget(self.radio_freq_label)
        layout.addWidget(self.radio_freq_spin)

        self.radio_gain_label = QLabel("Gain:")
        self.radio_gain_input = QLineEdit("10")
        self.radio_gain_input.setFixedWidth(46)
        self.radio_gain_input.setToolTip(
            'RTL tuner gain in dB, or "auto".\n'
            'Note: rtl_sdr treats 0 as "enable tuner AGC" (≈ max gain), not 0 dB.')
        layout.addWidget(self.radio_gain_label)
        layout.addWidget(self.radio_gain_input)

        self.radio_ppm_label = QLabel("PPM:")
        self.radio_ppm_spin = QSpinBox()
        self.radio_ppm_spin.setRange(-200, 200)
        self.radio_ppm_spin.setValue(0)
        self.radio_ppm_spin.setFixedWidth(62)
        layout.addWidget(self.radio_ppm_label)
        layout.addWidget(self.radio_ppm_spin)

        self.radio_rate_label = QLabel("Rate:")
        self.radio_rate_spin = QSpinBox()
        self.radio_rate_spin.setRange(48000, 2400000)
        self.radio_rate_spin.setSingleStep(48000)
        self.radio_rate_spin.setValue(DEFAULT_SAMPLE_RATE_HZ)
        self.radio_rate_spin.setFixedWidth(92)
        layout.addWidget(self.radio_rate_label)
        layout.addWidget(self.radio_rate_spin)

        self.radio_device_label = QLabel("Dev:")
        self.radio_device_input = QLineEdit("0")
        self.radio_device_input.setFixedWidth(36)
        layout.addWidget(self.radio_device_label)
        layout.addWidget(self.radio_device_input)

        # HIL controls — fake flight computer (pty, no hardware) + pacing
        self.hil_fake_check = QCheckBox("Fake FC")
        self.hil_fake_check.setToolTip(
            "Run against the in-process reference flight computer\n"
            "(apex_sim.hil.fake_teensy) instead of a real Teensy.")
        self.hil_fake_check.setChecked(True)
        self.hil_fake_check.toggled.connect(self._on_source_changed)
        layout.addWidget(self.hil_fake_check)

        self.hil_speed_label = QLabel("Speed:")
        self.hil_speed_spin = QDoubleSpinBox()
        self.hil_speed_spin.setRange(0.0, 4.0)
        self.hil_speed_spin.setDecimals(2)
        self.hil_speed_spin.setSingleStep(0.25)
        self.hil_speed_spin.setValue(1.0)
        self.hil_speed_spin.setFixedWidth(72)
        self.hil_speed_spin.setToolTip(
            "Sim pacing. 1.0 = real time (required for real hardware —\n"
            "the firmware's filter integrates wall-clock dt). 0 = max (fake only).")
        layout.addWidget(self.hil_speed_label)
        layout.addWidget(self.hil_speed_spin)

        self.hil_noise_check = QCheckBox("Noise")
        self.hil_noise_check.setChecked(True)
        self.hil_noise_check.setToolTip(
            "Per-sample sensor noise from the docs/sensors noise model\n"
            "(BMP581 0.32 Pa, ICM-45686 0.069 m/s², MMC5983MA 0.4 mG, ...).")
        layout.addWidget(self.hil_noise_check)

        self.hil_seed_label = QLabel("Seed:")
        self.hil_seed_input = QLineEdit()
        self.hil_seed_input.setPlaceholderText("auto")
        self.hil_seed_input.setFixedWidth(54)
        self.hil_seed_input.setToolTip(
            "Optional integer RNG seed for reproducible noisy HIL runs.\n"
            "Leave blank for a fresh random sequence.")
        layout.addWidget(self.hil_seed_label)
        layout.addWidget(self.hil_seed_input)

        self.hil_delay_label = QLabel("Delay:")
        self.hil_delay_spin = QDoubleSpinBox()
        self.hil_delay_spin.setRange(0.0, 500.0)
        self.hil_delay_spin.setDecimals(0)
        self.hil_delay_spin.setSingleStep(10.0)
        self.hil_delay_spin.setValue(0.0)
        self.hil_delay_spin.setSuffix(" ms")
        self.hil_delay_spin.setFixedWidth(74)
        self.hil_delay_spin.setToolTip(
            "Sensor transport delay injected before the flight computer sees\n"
            "each simulated sample. Rounded to 10 ms HIL ticks.")
        layout.addWidget(self.hil_delay_label)
        layout.addWidget(self.hil_delay_spin)

        self.hil_pad_label = QLabel("Pre:")
        self.hil_pad_spin = QDoubleSpinBox()
        self.hil_pad_spin.setRange(0.0, 1200.0)   # up to 20 min on the pad
        self.hil_pad_spin.setDecimals(0)
        self.hil_pad_spin.setSingleStep(5.0)
        self.hil_pad_spin.setValue(6.0)
        self.hil_pad_spin.setSuffix(" s")
        self.hil_pad_spin.setFixedWidth(90)
        self.hil_pad_spin.setToolTip(
            "Seconds the FC sits on the pad with arm switches closed before\n"
            "liftoff. The FC arms itself a few seconds in (its own\n"
            "IDLE→ARMED gate) and then sits ARMED on real sensor data for\n"
            "the rest of this time. At 1x this is real time — e.g. 1200 =\n"
            "a 20-minute wait on the pad before launch.")
        layout.addWidget(self.hil_pad_label)
        layout.addWidget(self.hil_pad_spin)

        self.hil_post_label = QLabel("Post:")
        self.hil_post_spin = QDoubleSpinBox()
        self.hil_post_spin.setRange(0.0, 1200.0)
        self.hil_post_spin.setDecimals(0)
        self.hil_post_spin.setSingleStep(5.0)
        self.hil_post_spin.setValue(5.0)
        self.hil_post_spin.setSuffix(" s")
        self.hil_post_spin.setFixedWidth(80)
        self.hil_post_spin.setToolTip(
            "Seconds of stationary post-touchdown samples to stream after a\n"
            "full-flight HIL landing. This exercises LANDED handling and\n"
            "post-landing storage behavior.")
        layout.addWidget(self.hil_post_label)
        layout.addWidget(self.hil_post_spin)

        self.hil_record_check = QCheckBox("Record CSV")
        self.hil_record_check.setChecked(True)
        self.hil_record_check.setToolTip(
            "Save each HIL run to sim/output/hil_gui_<timestamp>.csv.")
        layout.addWidget(self.hil_record_check)

        self.hil_full_check = QCheckBox("Full flight")
        self.hil_full_check.setChecked(False)
        self.hil_full_check.setToolTip(
            "Fly through descent and landing (DESCENT→LANDED, post-landing "
            "SD dump) instead of stopping at apogee.\n"
            "At 1× on real hardware this is several real-time minutes.")
        layout.addWidget(self.hil_full_check)

        self.hil_compare_check = QCheckBox("Compare Fake")
        self.hil_compare_check.setChecked(False)
        self.hil_compare_check.setToolTip(
            "When running real hardware HIL, mirror the same sensor packets\n"
            "through the in-process fake flight computer and plot/log deltas.")
        layout.addWidget(self.hil_compare_check)

        # Retune live: any radio setting change restarts rtl_sdr (debounced)
        self.radio_freq_spin.valueChanged.connect(self._schedule_radio_retune)
        self.radio_ppm_spin.valueChanged.connect(self._schedule_radio_retune)
        self.radio_rate_spin.valueChanged.connect(self._schedule_radio_retune)
        self.radio_gain_input.editingFinished.connect(self._schedule_radio_retune)
        self.radio_device_input.editingFinished.connect(self._schedule_radio_retune)

        layout.addSpacing(12)

        # Connect / disconnect
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setFixedWidth(90)
        self.connect_btn.clicked.connect(self._toggle_connection)
        layout.addWidget(self.connect_btn)

        layout.addSpacing(12)

        # Time window
        self.window_label = QLabel("Window:")
        layout.addWidget(self.window_label)
        self.window_spin = QSpinBox()
        self.window_spin.setRange(5, 300)
        self.window_spin.setValue(DEFAULT_WINDOW_S)
        self.window_spin.setSuffix(" s")
        self.window_spin.setFixedWidth(72)
        self.window_spin.valueChanged.connect(lambda v: setattr(self, "_window_s", v))
        layout.addWidget(self.window_spin)

        layout.addSpacing(12)

        # Clear button — same width as Connect so the toolbar reads as a unit
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.setFixedWidth(90)
        self.clear_btn.clicked.connect(self._clear_data)
        layout.addWidget(self.clear_btn)

        # Logs page has no live connection — the toolbar states the pipeline
        # instead of showing orphaned controls.
        self.logs_hint_label = QLabel(
            "Flight Computer  →  Laptop Archive  →  CSV Exports")
        self.logs_hint_label.setFont(mono_font(9))
        self.logs_hint_label.setStyleSheet(f"color:{TEXT_DIM};")
        layout.addWidget(self.logs_hint_label)

        layout.addStretch()

        # Status indicator
        self.status_label = QLabel("Not connected")
        self.status_label.setFont(mono_font(9))
        self.status_label.setStyleSheet(f"color:{TEXT_DIM};")
        layout.addWidget(self.status_label)

        self._on_source_changed()
        return bar

    def _build_plot_panel(self) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        container = QWidget()
        self._plot_layout = QVBoxLayout(container)
        self._plot_layout.setContentsMargins(0, 0, 0, 0)
        self._plot_layout.setSpacing(4)

        self._plot_groups: list[PlotGroupWidget] = []

        # Spectrum + waterfall — only visible when the RTL-SDR source is active
        self.spectrum_panel = SpectrumPanel()
        self.spectrum_panel.setVisible(self._source_mode() == "radio")
        self._plot_layout.addWidget(self.spectrum_panel)

        # Two layouts share the panel: full sensor groups for USB serial, a
        # ground-station set for the radio source. Visibility tracks the source.
        radio = self._source_mode() == "radio"

        self._serial_groups: list[PlotGroupWidget] = []
        for title, keys, colors in PLOT_GROUPS:
            group = PlotGroupWidget(title, keys, colors)
            group.setVisible(not radio)
            self._serial_groups.append(group)
            self._plot_groups.append(group)
            self._plot_layout.addWidget(group)
            for key in keys:
                self._key_to_group_serial[key] = group

        self._radio_groups: list[PlotGroupWidget] = []
        for title, keys, colors in RADIO_PLOT_GROUPS:
            group = PlotGroupWidget(title, keys, colors)
            group.setVisible(radio)
            self._radio_groups.append(group)
            self._plot_groups.append(group)
            self._plot_layout.addWidget(group)
            for key in keys:
                self._key_to_group_radio[key] = group

        hil = self._source_mode() == "hil"
        self._hil_groups: list[PlotGroupWidget] = []
        for title, keys, colors in HIL_PLOT_GROUPS:
            group = PlotGroupWidget(title, keys, colors)
            group.setVisible(hil)
            self._hil_groups.append(group)
            self._plot_groups.append(group)
            self._plot_layout.addWidget(group)
            for key in keys:
                self._key_to_group_hil[key] = group

        self.log_decode_panel = self._build_log_decode_panel()
        self.log_decode_panel.setVisible(self._source_mode() == "logs")
        self._plot_layout.addWidget(self.log_decode_panel)

        self._plot_layout.addStretch()
        scroll.setWidget(container)
        return scroll

    # Shared inset-table stylesheet for the Logs page (device + flights tables)
    def _logs_table_style(self) -> str:
        return (
            f"QTableWidget {{ background:{INSET}; color:{TEXT};"
            f"  border:1px solid {BORDER}; gridline-color:{BORDER}; }}"
            f"QHeaderView::section {{ background:{SURFACE}; color:{TEXT_DIM};"
            f"  border:none; padding:2px 6px; }}")

    @staticmethod
    def _fmt_size(n) -> str:
        if not isinstance(n, (int, float)):
            return "—"
        if n >= 1024 * 1024:
            return f"{n / (1024 * 1024):.1f} MiB"
        if n >= 1024:
            return f"{n / 1024:.1f} KiB"
        return f"{int(n)} B"

    def _build_log_decode_panel(self) -> QWidget:
        """Logs page — a top-to-bottom device-file pipeline in three sections:
        files on the flight computer → raw binaries archived on the laptop →
        decoded CSV exports."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        def section_hint(text: str) -> QLabel:
            hint = QLabel(text)
            hint.setFont(mono_font(8))
            hint.setStyleSheet(f"color:{TEXT_DIM};")
            return hint

        # ── 1 · Flight Computer — .APXLOG files present on the device ───────
        device_box = QGroupBox("1 · Flight Computer  —  files on the device")
        device_layout = QVBoxLayout(device_box)
        device_layout.setContentsMargins(8, 6, 8, 8)
        device_layout.setSpacing(6)
        device_layout.addWidget(section_hint(
            "Raw .APXLOG files on the Teensy's storage. Refresh lists the FC "
            "over MTP and marks which files are already archived locally. Pull "
            "actions skip archived files and only transfer missing APXLOGs."))

        self.device_capacity_label = QLabel("Capacity: —")
        self.device_capacity_label.setFont(mono_font(8))
        self.device_capacity_label.setStyleSheet(f"color:{TEXT_DIM};")
        self.device_capacity_label.setWordWrap(True)
        device_layout.addWidget(self.device_capacity_label)

        self.device_table = QTableWidget(0, 4)
        self.device_table.setHorizontalHeaderLabels(["File", "Size", "Local", "Source"])
        self.device_table.verticalHeader().setVisible(False)
        self.device_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.device_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.device_table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.device_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.device_table.setFont(mono_font(9))
        self.device_table.setFixedHeight(140)
        self.device_table.setStyleSheet(self._logs_table_style())
        device_layout.addWidget(self.device_table)

        device_row = QHBoxLayout()
        device_row.setSpacing(6)
        refresh_device_btn = QPushButton("Refresh")
        refresh_device_btn.setFixedWidth(110)
        refresh_device_btn.clicked.connect(self._refresh_device_files)
        device_row.addWidget(refresh_device_btn)
        device_row.addWidget(QLabel("Storage:"))
        self.device_filter_combo = QComboBox()
        self.device_filter_combo.addItem("All", "all")
        self.device_filter_combo.addItem("QSPI", "qspi")
        self.device_filter_combo.addItem("SD", "sd")
        self.device_filter_combo.setFixedWidth(100)
        self.device_filter_combo.currentIndexChanged.connect(
            self._render_device_table)
        device_row.addWidget(self.device_filter_combo)
        select_all_device_btn = QPushButton("Select All")
        select_all_device_btn.setFixedWidth(110)
        select_all_device_btn.clicked.connect(self.device_table.selectAll)
        device_row.addWidget(select_all_device_btn)
        device_row.addStretch()
        pull_sel_btn = QPushButton("Pull Selected Missing")
        pull_sel_btn.setFixedWidth(170)
        pull_sel_btn.clicked.connect(self._pull_selected_device_files)
        device_row.addWidget(pull_sel_btn)
        pull_all_btn = QPushButton("Pull All Missing")
        pull_all_btn.setFixedWidth(150)
        pull_all_btn.clicked.connect(lambda: self._pull_all_device_files(export_after=False))
        device_row.addWidget(pull_all_btn)
        pull_all_export_btn = QPushButton("Pull Missing + Export")
        pull_all_export_btn.setFixedWidth(180)
        pull_all_export_btn.clicked.connect(lambda: self._pull_all_device_files(export_after=True))
        device_row.addWidget(pull_all_export_btn)
        delete_device_btn = QPushButton("Delete Selected")
        delete_device_btn.setFixedWidth(130)
        delete_device_btn.setToolTip(
            "Delete selected APXLOG files from the flight computer.\n"
            "Requires local-copy checks and typed confirmation.")
        delete_device_btn.clicked.connect(self._delete_selected_device_files)
        device_row.addWidget(delete_device_btn)
        format_qspi_btn = QPushButton("Format QSPI")
        format_qspi_btn.setFixedWidth(120)
        format_qspi_btn.setToolTip(
            "Erase and reformat the flight computer's QSPI flash.\n"
            "Requires local-copy checks and typed confirmation.")
        format_qspi_btn.clicked.connect(self._format_qspi_flash)
        device_row.addWidget(format_qspi_btn)
        device_layout.addLayout(device_row)
        layout.addWidget(device_box)

        # ── 2 · Laptop Archive — raw binaries pulled onto this machine ──────
        archive_box = QGroupBox("2 · Laptop Archive  —  raw .APXLOG binaries on this laptop")
        archive_layout = QVBoxLayout(archive_box)
        archive_layout.setContentsMargins(8, 6, 8, 8)
        archive_layout.setSpacing(6)

        archive_row = QHBoxLayout()
        archive_row.setSpacing(6)
        self.log_archive_label = QLabel(str(_RAW_LOG_ARCHIVE))
        self.log_archive_label.setFont(mono_font(9))
        self.log_archive_label.setStyleSheet(f"color:{TEXT_DIM};")
        archive_row.addWidget(self.log_archive_label, stretch=1)

        refresh_archive_btn = QPushButton("Refresh")
        refresh_archive_btn.setFixedWidth(110)
        refresh_archive_btn.clicked.connect(lambda: self._refresh_local_log_choices(select_all=False))
        archive_row.addWidget(refresh_archive_btn)

        select_all_btn = QPushButton("Select All")
        select_all_btn.setFixedWidth(110)
        select_all_btn.clicked.connect(self._select_all_local_logs)
        archive_row.addWidget(select_all_btn)

        add_external_btn = QPushButton("Add external…")
        add_external_btn.setFixedWidth(110)
        add_external_btn.setToolTip(
            "Copy .APXLOG files from anywhere on disk into the archive\n"
            "(e.g. logs someone sent you).")
        add_external_btn.clicked.connect(self._add_external_logs)
        archive_row.addWidget(add_external_btn)
        delete_local_btn = QPushButton("Delete Selected Local")
        delete_local_btn.setFixedWidth(150)
        delete_local_btn.setToolTip(
            "Move selected local APXLOG copies to raw_logs_deleted.\n"
            "Requires device-copy checks and typed confirmation.")
        delete_local_btn.clicked.connect(self._delete_selected_local_logs)
        archive_row.addWidget(delete_local_btn)
        archive_layout.addLayout(archive_row)
        archive_layout.addWidget(section_hint(
            "FC pulls are saved under output/raw_logs/flight_computer by "
            "storage/parent/file name. HORIZON still recognizes older archive "
            "copies anywhere under raw_logs and sorts by first valid UTC."))

        self.local_log_list = QListWidget()
        self.local_log_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.local_log_list.setMinimumHeight(110)
        archive_layout.addWidget(self.local_log_list)

        export_row = QHBoxLayout()
        export_row.setSpacing(6)
        export_row.addStretch()
        export_sel_btn = QPushButton("Export Selected to CSV")
        export_sel_btn.setFixedWidth(170)
        export_sel_btn.clicked.connect(lambda: self._export_logs(all_files=False))
        export_row.addWidget(export_sel_btn)
        export_all_btn = QPushButton("Export All")
        export_all_btn.setFixedWidth(130)
        export_all_btn.clicked.connect(lambda: self._export_logs(all_files=True))
        export_row.addWidget(export_all_btn)
        archive_layout.addLayout(export_row)
        layout.addWidget(archive_box)

        # ── 3 · CSV Exports — the decoded, final results ─────────────────────
        exports_box = QGroupBox("3 · CSV Exports  —  decoded flight data (final CSVs)")
        exports_layout = QVBoxLayout(exports_box)
        exports_layout.setContentsMargins(8, 6, 8, 8)
        exports_layout.setSpacing(6)

        output_row = QHBoxLayout()
        output_row.setSpacing(6)
        out_label = QLabel("Export folder:")
        out_label.setFont(mono_font(9))
        out_label.setStyleSheet(f"color:{TEXT_DIM};")
        output_row.addWidget(out_label)
        self.log_output_field = QLineEdit(str(_SIM_ROOT / "output" / "log_exports"))
        output_row.addWidget(self.log_output_field, stretch=1)
        output_btn = QPushButton("Choose")
        output_btn.setFixedWidth(110)
        output_btn.clicked.connect(self._browse_log_output)
        output_row.addWidget(output_btn)
        open_folder_btn = QPushButton("Open Export Folder")
        open_folder_btn.setFixedWidth(150)
        open_folder_btn.clicked.connect(self._open_export_folder)
        output_row.addWidget(open_folder_btn)
        exports_layout.addLayout(output_row)

        # Per-flight summary of the last decode — one CSV per flight.
        self.flights_table = QTableWidget(0, 7)
        self.flights_table.setHorizontalHeaderLabels(
            ["Flight", "Boot", "Records", "Start (UTC)", "Duration",
             "Max alt", "Events"])
        self.flights_table.verticalHeader().setVisible(False)
        self.flights_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.flights_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.flights_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.flights_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.flights_table.setFont(mono_font(9))
        self.flights_table.setFixedHeight(150)
        self.flights_table.setStyleSheet(self._logs_table_style())
        exports_layout.addWidget(self.flights_table)
        layout.addWidget(exports_box)

        # Operations log — progress lines from the worker, kept small
        ops_row = QHBoxLayout()
        ops_label = QLabel("Operations log")
        ops_label.setFont(mono_font(8))
        ops_label.setStyleSheet(f"color:{TEXT_DIM};")
        ops_row.addWidget(ops_label)
        self.log_busy_label = QLabel("")
        self.log_busy_label.setFont(mono_font(9))
        self.log_busy_label.setStyleSheet(f"color:{AMBER};")
        ops_row.addWidget(self.log_busy_label)
        ops_row.addStretch()
        layout.addLayout(ops_row)

        self.log_decode_view = QTextEdit()
        self.log_decode_view.setReadOnly(True)
        self.log_decode_view.setFont(mono_font(9))
        self.log_decode_view.setMaximumHeight(130)
        self.log_decode_view.setStyleSheet(
            f"background:{INSET}; color:{TEXT}; border:1px solid {BORDER};")
        layout.addWidget(self.log_decode_view, stretch=1)

        self._log_action_buttons = [
            refresh_device_btn, select_all_device_btn, pull_sel_btn,
            pull_all_btn, pull_all_export_btn, delete_device_btn,
            format_qspi_btn, self.device_filter_combo,
            refresh_archive_btn, select_all_btn, add_external_btn,
            delete_local_btn, export_sel_btn, export_all_btn,
        ]
        self._refresh_local_log_choices(select_all=False)
        return panel

    def _populate_flights_table(self, rows: list):
        self.flights_table.setRowCount(len(rows))
        for i, (label, bid, n, utc, dur_s, max_alt, events) in enumerate(rows):
            alt_txt = (f"{max_alt * FT_PER_M:,.0f} ft" if max_alt is not None
                       else "—")
            cells = [label, str(bid), str(n), utc or "—", f"{dur_s:,.1f} s",
                     alt_txt, str(events)]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col >= 1:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.flights_table.setItem(i, col, item)

    def _open_export_folder(self):
        out = self._last_export_dir or Path(
            self.log_output_field.text().strip() or (_SIM_ROOT / "output" / "log_exports"))
        if not out.exists():
            self._log(f"[horizon] Export folder does not exist yet: {out}")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(out)))

    def _set_log_ops_busy(self, busy: bool, msg: str = ""):
        for btn in self._log_action_buttons:
            btn.setEnabled(not busy)
        self.log_busy_label.setText(msg if busy else "")

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self.state_panel = StatePanel()
        layout.addWidget(self.state_panel)

        log_box = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(4, 4, 4, 4)
        log_layout.setSpacing(2)

        log_toolbar = QHBoxLayout()
        log_toolbar.addStretch()

        copy_log_btn = QPushButton("Copy")
        copy_log_btn.setFixedWidth(48)
        copy_log_btn.setToolTip("Copy all log text to clipboard")
        copy_log_btn.clicked.connect(
            lambda: QApplication.clipboard().setText(self.log_view.toPlainText()))
        log_toolbar.addWidget(copy_log_btn)

        clear_log_btn = QPushButton("Clear")
        clear_log_btn.setFixedWidth(48)
        clear_log_btn.clicked.connect(lambda: self.log_view.clear())
        log_toolbar.addWidget(clear_log_btn)

        log_layout.addLayout(log_toolbar)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(mono_font(8))
        self.log_view.setStyleSheet(f"background:{INSET}; color:#cccccc; border:none;")
        self.log_view.document().setMaximumBlockCount(2000)
        log_layout.addWidget(self.log_view)

        # Command input row
        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(4)
        self.cmd_input = CommandInput(KNOWN_COMMANDS)
        self.cmd_input.setPlaceholderText("Command (Tab completes, Up/Down history)")
        self.cmd_input.setFont(mono_font(8))
        self.cmd_input.setStyleSheet(
            f"background:{INSET}; color:#ccffcc; border:1px solid {BORDER};"
            " border-radius:3px; padding:2px 4px;")
        self.cmd_input.returnPressed.connect(self._send_command)
        cmd_row.addWidget(self.cmd_input)
        send_btn = QPushButton("Send")
        send_btn.setFixedWidth(48)
        send_btn.clicked.connect(self._send_command)
        cmd_row.addWidget(send_btn)
        log_layout.addLayout(cmd_row)

        layout.addWidget(log_box, stretch=1)

        return panel

    # ── Signals ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self._worker.line_received.connect(self._on_line)
        self._worker.status_changed.connect(self._on_status)
        self._worker.reconnected.connect(self._on_reconnect)
        self._radio_worker.line_received.connect(self._on_line)
        self._radio_worker.line_received_at.connect(self._on_line)
        self._radio_worker.status_changed.connect(self._on_status)
        self._radio_worker.spectrum_ready.connect(self.spectrum_panel.update_spectrum)
        self._radio_worker.finished.connect(self._on_worker_finished)
        self._hil_worker.lines_received.connect(self._on_lines)
        self._hil_worker.status_changed.connect(self._on_status)
        self._hil_worker.finished.connect(self._on_worker_finished)
        self._log_ops.progress.connect(self._append_log_decode_text)
        self._log_ops.done.connect(self._on_log_job_done)
        self._log_ops.failed.connect(self._on_log_job_failed)
        self.state_panel.copy_requested.connect(self._copy_to_clipboard)

    # ── Serial control ────────────────────────────────────────────────────────

    def _source_mode(self) -> str:
        idx = self.tab_bar.currentIndex() if hasattr(self, "tab_bar") else 0
        return PAGES[idx][1] if 0 <= idx < len(PAGES) else "serial"

    def _on_source_changed(self):
        mode = self._source_mode()
        radio = mode == "radio"
        hil = mode == "hil"
        logs = mode == "logs"
        hil_fake = hil and self.hil_fake_check.isChecked()

        # Port selector serves USB serial, HIL-on-hardware, and the Logs tab
        # (which connects to the flight computer over the same serial link).
        for widget in (self.port_label, self.port_combo, self.refresh_btn):
            widget.setVisible(mode == "serial" or logs or (hil and not hil_fake))
        for widget in (self.baud_label, self.baud_combo):
            widget.setVisible(mode == "serial" or logs)
        radio_widgets = [
            self.radio_freq_label, self.radio_freq_spin,
            self.radio_gain_label, self.radio_gain_input,
            self.radio_ppm_label, self.radio_ppm_spin,
            self.radio_rate_label, self.radio_rate_spin,
            self.radio_device_label, self.radio_device_input,
        ]
        for widget in radio_widgets:
            widget.setVisible(radio)
        for widget in (self.hil_fake_check, self.hil_speed_label,
                       self.hil_speed_spin, self.hil_noise_check,
                       self.hil_seed_label, self.hil_seed_input,
                       self.hil_delay_label, self.hil_delay_spin,
                       self.hil_pad_label, self.hil_pad_spin,
                       self.hil_post_label, self.hil_post_spin,
                       self.hil_record_check, self.hil_full_check):
            widget.setVisible(hil)
        self.hil_compare_check.setVisible(hil and not hil_fake)

        if hasattr(self, "spectrum_panel"):
            self.spectrum_panel.setVisible(radio)
            for group in self._serial_groups:
                group.setVisible(mode == "serial")
            for group in self._radio_groups:
                group.setVisible(radio)
            for group in self._hil_groups:
                group.setVisible(hil)
            self.log_decode_panel.setVisible(logs)

        if hasattr(self, "state_panel"):
            self.state_panel.setVisible(not logs)
            self.state_panel.set_link_mode("hil" if hil else "radio" if radio else "serial")
        if logs and hasattr(self, "local_log_list"):
            self._refresh_local_log_choices(select_all=False)

        # Connect is available on every tab (the FC link is not tab-specific);
        # the plot-window controls are not meaningful on the Logs tab.
        self.connect_btn.setVisible(True)
        for widget in (self.window_label, self.window_spin, self.clear_btn):
            widget.setVisible(not logs)
        if hasattr(self, "logs_hint_label"):
            self.logs_hint_label.setVisible(logs)

        if hasattr(self, "cmd_input"):
            self.cmd_input.setEnabled(mode == "serial")
            if radio:
                self.cmd_input.setPlaceholderText("Radio RX is receive-only; send RADIO_DATA_TEST over USB")
            elif hil:
                self.cmd_input.setPlaceholderText("HIL link is packet-based — no text commands")
            elif logs:
                self.cmd_input.setPlaceholderText("Log export uses the Logs tab controls")
            else:
                self.cmd_input.setPlaceholderText("Command (Tab completes, Up/Down history)")

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda x: x.device)
        for p in ports:
            self.port_combo.addItem(f"{p.device}  —  {p.description}", p.device)
        self._auto_select_port(ports)

    def _auto_select_port(self, ports=None):
        """Select the most likely Teensy port automatically."""
        if ports is None:
            ports = list(serial.tools.list_ports.comports())

        TEENSY_VID = 0x16C0

        def score(p):
            # Highest priority: known Teensy VID
            if p.vid == TEENSY_VID:
                return 3
            # USB modem / ACM devices (common for Teensy, Arduino)
            dev = p.device.lower()
            if "usbmodem" in dev or "usbserial" in dev or "acm" in dev:
                return 2
            # Any USB serial
            if "usb" in dev or "cu." in dev:
                return 1
            return 0

        best = max(ports, key=score, default=None)
        if best is None or score(best) == 0:
            return

        for i in range(self.port_combo.count()):
            if self.port_combo.itemData(i) == best.device:
                self.port_combo.setCurrentIndex(i)
                return

    def _toggle_connection(self):
        if self._worker.isRunning():
            self._worker.stop()
            self._set_disconnected()
        elif self._radio_worker.isRunning():
            self._radio_worker.stop()
            self._set_disconnected()
        elif self._hil_worker.isRunning():
            self._hil_worker.stop()
            self._set_disconnected()
        else:
            mode = self._source_mode()
            if mode == "radio":
                self._start_radio()
            elif mode == "hil":
                self._start_hil()
            else:
                # serial OR logs — both talk to the flight computer over USB
                # serial. The connection is independent of which tab is showing,
                # so the FC can be connected from the Logs tab too (format/ARM).
                idx  = self.port_combo.currentIndex()
                port = self.port_combo.itemData(idx) or self.port_combo.currentText().split()[0]
                baud = int(self.baud_combo.currentText())
                self._worker.configure(port, baud)
                self._worker.start()
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet(
                f"background:{BAD}; color:#ffffff; border:1px solid #cc4444;")

    def _start_hil(self):
        fake = self.hil_fake_check.isChecked()
        speed = float(self.hil_speed_spin.value())
        seed_text = self.hil_seed_input.text().strip()
        sensor_seed = None
        seed_warning = None
        if seed_text:
            try:
                sensor_seed = int(seed_text, 0)
            except ValueError:
                seed_warning = f"[horizon] Invalid HIL seed '{seed_text}' — using auto"
        delay_ms = float(self.hil_delay_spin.value())
        record_csv = self.hil_record_check.isChecked()
        compare_fake = (not fake) and self.hil_compare_check.isChecked()
        full_flight = self.hil_full_check.isChecked()
        pad_time = float(self.hil_pad_spin.value())
        post_landed_time = float(self.hil_post_spin.value())
        port = ""
        self._clear_data()
        if seed_warning is not None:
            self._log(seed_warning)
        if not fake:
            idx = self.port_combo.currentIndex()
            port = self.port_combo.itemData(idx) or self.port_combo.currentText().split()[0]
            if speed != 1.0:
                self._log("[horizon] HIL on real hardware needs speed 1.0 — overriding "
                          "(firmware filter integrates wall-clock dt)")
                speed = 1.0
        self._log(
            "[horizon] HIL options: "
            f"{'fake FC' if fake else 'real FC'}, speed={speed:.2f}, "
            f"noise={'on' if self.hil_noise_check.isChecked() else 'off'}, "
            f"seed={sensor_seed if sensor_seed is not None else 'auto'}, "
            f"delay={delay_ms:.0f} ms, "
            f"csv={'on' if record_csv else 'off'}, "
            f"compare_fake={'on' if compare_fake else 'off'}, "
            f"full_flight={'on' if full_flight else 'off'}, "
            f"pre_launch={pad_time:.0f}s, "
            f"post_landed={post_landed_time:.0f}s")
        self._hil_worker.configure(
            port, fake, speed,
            noise=self.hil_noise_check.isChecked(),
            record_csv=record_csv,
            compare_fake=compare_fake,
            sensor_seed=sensor_seed,
            sensor_delay_ms=delay_ms,
            full_flight=full_flight,
            pad_time=pad_time,
            post_landed_time=post_landed_time)
        self._hil_worker.start()

    def _start_radio(self):
        freq_hz = int(round(self.radio_freq_spin.value() * 1e6))
        sample_rate = int(self.radio_rate_spin.value())
        gain = self.radio_gain_input.text().strip() or "auto"
        ppm = int(self.radio_ppm_spin.value())
        device = self.radio_device_input.text().strip() or "0"
        self._radio_worker.configure(freq_hz, sample_rate, gain, ppm, device)
        self.spectrum_panel.set_params(freq_hz, sample_rate)
        self._radio_worker.start()

    def _schedule_radio_retune(self):
        if self._radio_worker.isRunning():
            self._retune_timer.start()

    def _apply_radio_retune(self):
        if not self._radio_worker.isRunning():
            return
        self._log("[horizon] Radio settings changed — retuning…")
        self._radio_worker.stop()
        self._start_radio()

    # ── Log decode/export ─────────────────────────────────────────────────────

    def _set_log_decode_text(self, text: str):
        self.log_decode_view.setPlainText(text)
        self.log_decode_view.verticalScrollBar().setValue(
            self.log_decode_view.verticalScrollBar().maximum())

    def _append_log_decode_text(self, text: str):
        current = self.log_decode_view.toPlainText()
        self._set_log_decode_text((current + "\n" if current else "") + text)

    def _local_log_paths(self) -> list[Path]:
        if not _RAW_LOG_ARCHIVE.exists():
            return []
        return sorted(
            path for path in _RAW_LOG_ARCHIVE.rglob("*")
            if path.is_file() and path.suffix.upper() == ".APXLOG"
        )

    def _local_log_info(self, path: Path) -> dict:
        try:
            stat = path.stat()
            signature = (stat.st_size, stat.st_mtime_ns)
        except OSError:
            signature = (0, 0)
        cache_key = str(path)
        cached = self._local_log_info_cache.get(cache_key)
        if cached and cached[0] == signature:
            return cached[1]

        info = {
            "path": path,
            "size": signature[0],
            "records": 0,
            "boot_ids": [],
            "flight_ids": [],
            "first_utc": None,
            "sort_dt": None,
            "sort_fallback": signature[1],
            "decode_error": None,
        }
        try:
            from apex_sim.logs.decoder import REC_SAMPLE, decode_file
            records, stats = decode_file(path)
            info["records"] = stats.records
            info["boot_ids"] = sorted({r.boot_id for r in records})
            info["flight_ids"] = sorted({r.flight_id for r in records if r.flight_id})
            for record in records:
                if record.record_type != REC_SAMPLE:
                    continue
                utc = record.payload.get("utc")
                if not utc:
                    continue
                info["first_utc"] = utc
                try:
                    info["sort_dt"] = datetime.strptime(utc, "%Y-%m-%dT%H:%M:%S.%fZ")
                except ValueError:
                    pass
                break
        except Exception as exc:  # noqa: BLE001 — shown in item tooltip
            info["decode_error"] = str(exc)

        self._local_log_info_cache[cache_key] = (signature, info)
        return info

    def _format_local_log_item(self, path: Path, info: dict) -> str:
        try:
            rel = path.relative_to(_RAW_LOG_ARCHIVE)
        except ValueError:
            rel = path
        if info.get("first_utc"):
            start = str(info["first_utc"]).replace("T", " ").replace(".000Z", "Z")
        else:
            start = "no UTC in data"

        boot_ids = info.get("boot_ids") or []
        flight_ids = info.get("flight_ids") or []
        boot = "boot " + ",".join(str(b) for b in boot_ids) if boot_ids else "boot ?"
        flights = "flight " + ",".join(str(f) for f in flight_ids) if flight_ids else "ground"
        size_txt = self._fmt_size(info.get("size", 0))
        return f"{start}  —  {boot}  —  {flights}  —  {rel}  —  {size_txt}"

    def _refresh_local_log_choices(self, select_all: bool = False,
                                   focus_paths: list[Path] | None = None) -> int:
        focus = {str(path.resolve()) for path in (focus_paths or [])}
        if not focus and not select_all and hasattr(self, "local_log_list"):
            focus = {
                str(Path(item.data(Qt.UserRole)).resolve())
                for item in self.local_log_list.selectedItems()
            }

        paths = self._local_log_paths()
        infos = [(path, self._local_log_info(path)) for path in paths]
        infos.sort(
            key=lambda item: (
                item[1].get("sort_dt") is not None,
                item[1].get("sort_dt") or datetime.fromtimestamp(
                    item[1].get("sort_fallback", 0) / 1_000_000_000),
                item[0].name,
            ),
            reverse=True,
        )
        self.local_log_list.blockSignals(True)
        self.local_log_list.clear()
        for path, info in infos:
            item = QListWidgetItem(self._format_local_log_item(path, info))
            item.setData(Qt.UserRole, str(path))
            tooltip = [
                str(path),
                f"records: {info.get('records', 0)}",
                f"boot_ids: {info.get('boot_ids') or 'unknown'}",
                f"flight_ids: {info.get('flight_ids') or 'none'}",
                f"first_utc: {info.get('first_utc') or 'not found'}",
            ]
            if info.get("decode_error"):
                tooltip.append(f"decode_error: {info['decode_error']}")
            item.setToolTip("\n".join(str(x) for x in tooltip))
            self.local_log_list.addItem(item)
            if select_all or str(path.resolve()) in focus:
                item.setSelected(True)
        self.local_log_list.blockSignals(False)
        if hasattr(self, "device_table") and self._device_entries:
            self._populate_device_table(self._device_entries)
        return len(infos)

    def _select_all_local_logs(self):
        if self.local_log_list.count() == 0:
            self._refresh_local_log_choices(select_all=False)
        self.local_log_list.selectAll()

    def _selected_local_log_paths(self) -> list[Path]:
        return [Path(item.data(Qt.UserRole))
                for item in self.local_log_list.selectedItems()]

    def _local_log_index(self) -> dict[tuple[str, int], list[Path]]:
        index: dict[tuple[str, int], list[Path]] = {}
        for path in self._local_log_paths():
            try:
                size = path.stat().st_size
            except OSError:
                continue
            index.setdefault((path.name, size), []).append(path)
        return index

    @staticmethod
    def _safe_archive_part(value: object, fallback: str) -> str:
        text = str(value or fallback).strip() or fallback
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", text)

    def _entry_archive_path(self, entry: dict, archive: Path = _RAW_LOG_ARCHIVE) -> Path:
        filename = Path(str(entry.get("name") or entry.get("filename") or "unknown.APXLOG")).name
        if entry.get("kind") == "mtp":
            storage = self._safe_archive_part(entry.get("storage_id"), "device")
            parent = self._safe_archive_part(entry.get("parent_id"), "root")
            return archive / "flight_computer" / f"storage_{storage}" / f"parent_{parent}" / filename

        src = Path(str(entry.get("src", filename)))
        root = Path(str(entry.get("root", src.parent)))
        try:
            rel = src.relative_to(root)
        except ValueError:
            rel = Path(filename)
        volume = self._safe_archive_part(root.parent.name, "mounted_volume")
        return archive / "flight_computer" / volume / rel

    def _find_local_copy_for_entry(self, entry: dict,
                                   index: dict[tuple[str, int], list[Path]] | None = None) -> Path | None:
        size = entry.get("size")
        expected = self._entry_archive_path(entry)
        if expected.exists():
            if not isinstance(size, int) or expected.stat().st_size == size:
                return expected
        if not isinstance(size, int):
            return None
        name = Path(str(entry.get("name", ""))).name
        matches = (index or self._local_log_index()).get((name, size), [])
        return matches[0] if matches else None

    def _annotate_device_entries(self, entries: list[dict]) -> list[dict]:
        local_index = self._local_log_index()
        annotated: list[dict] = []
        for entry in entries:
            item = dict(entry)
            archive_path = self._entry_archive_path(item)
            local_copy = self._find_local_copy_for_entry(item, local_index)
            item["archive_path"] = str(archive_path)
            item["local_path"] = str(local_copy) if local_copy else ""
            item["local_status"] = "archived" if local_copy else "missing"
            annotated.append(item)
        return annotated

    def _missing_device_entries(self, entries: list[dict]) -> list[dict]:
        local_index = self._local_log_index()
        return [entry for entry in entries
                if self._find_local_copy_for_entry(entry, local_index) is None]

    def _device_entry_matches_local_path(self, entry: dict, path: Path) -> bool:
        try:
            size = path.stat().st_size
        except OSError:
            return False
        return Path(str(entry.get("name", ""))).name == path.name and entry.get("size") == size

    @staticmethod
    def _device_entry_key(entry: dict) -> tuple:
        if entry.get("kind") == "mtp":
            return ("mtp", entry.get("id"))
        return ("volume", entry.get("src"))

    def _confirm_dangerous_delete(self, title: str, body: str,
                                  extra_warning: str = "",
                                  phrase: str = _DELETE_CONFIRM_PHRASE) -> bool:
        lines = [
            body,
            "",
            "This cannot happen from one click. The next step requires an exact typed confirmation.",
        ]
        if extra_warning:
            lines.extend(["", extra_warning])
        lines.extend([
            "",
            f'Type exactly: "{phrase}"',
        ])
        QMessageBox.warning(self, title, "\n".join(lines))
        text, ok = QInputDialog.getText(
            self, title, f'Type exactly:\n{phrase}')
        if not ok:
            return False
        return text.strip() == phrase

    def _add_external_logs(self):
        """Copy .APXLOG files from anywhere on disk into the laptop archive."""
        files, _ = QFileDialog.getOpenFileNames(
            self, "Add external Apex binary logs to the archive", str(_SIM_ROOT),
            "Apex logs (*.APXLOG *.apxlog);;All files (*)")
        if not files:
            return
        added: list[Path] = []
        skipped = 0
        dest_root = _RAW_LOG_ARCHIVE / "external"
        dest_root.mkdir(parents=True, exist_ok=True)
        for item in files:
            src = Path(item)
            dst = dest_root / src.name
            if dst.exists() and dst.stat().st_size == src.stat().st_size:
                skipped += 1
                continue
            dst = self._dedupe_path(dst)
            shutil.copy2(src, dst)
            added.append(dst)
        self._log(f"[horizon] Added {len(added)} external log(s) to the archive"
                  + (f", {skipped} already present" if skipped else ""))
        self._refresh_local_log_choices(focus_paths=added or None)

    def _browse_log_output(self):
        folder = QFileDialog.getExistingDirectory(
            self, "Select log export folder", self.log_output_field.text().strip() or str(_SIM_ROOT))
        if folder:
            self.log_output_field.setText(folder)

    @staticmethod
    def _dedupe_path(dst: Path) -> Path:
        """Free destination path — appends _1, _2, … if dst already exists."""
        if not dst.exists():
            return dst
        stem, suffix = dst.stem, dst.suffix
        n = 1
        while True:
            alt = dst.with_name(f"{stem}_{n}{suffix}")
            if not alt.exists():
                return alt
            n += 1

    # ── Device file listing / per-file pulls (run inside LogOpsWorker) ──────
    # Entries are plain dicts so job closures never touch Qt objects:
    #   volume: {"kind","name","size","src","root","source"}
    #   mtp:    {"kind","name","size","id","parent_id","storage_id","source"}

    def _list_device_entries(self, progress) -> tuple[str, list[dict], str, list[dict]]:
        """Worker-thread helper: list .APXLOG files present on the device.
        libmtp only. Returns
        (mode, entries, raw_mtp_listing, capacity rows)."""

        progress("Listing via libmtp (close OpenMTP first)…")
        code, listing = self._run_mtp_tool(["mtp-files"], timeout_s=30)
        mtp_failure = self._mtp_listing_failure(listing)
        if mtp_failure:
            progress("libmtp did not find a usable MTP device.")
            raise RuntimeError(mtp_failure)
        if code == 0:
            capacity = self._mtp_capacity(progress)
            entries = []
            for e in self._parse_mtp_files_output(listing):
                storage_id = e.get("storage_id")
                entries.append({
                    "kind": "mtp",
                    "name": str(e.get("filename", e.get("id"))),
                    "size": e.get("size"),
                    "id": e.get("id"),
                    "parent_id": e.get("parent_id"),
                    "storage_id": storage_id,
                    "source": self._mtp_storage_label(storage_id),
                })
            if entries:
                return "libmtp", entries, listing, capacity
            progress("libmtp listed no APXLOG files.")
            return "libmtp", entries, listing, capacity
        else:
            mtp_error = listing if code is None else (
                listing.strip() or f"mtp-files exited with status {code}")
            progress("libmtp did not list files.")
            raise RuntimeError(mtp_error)

    @staticmethod
    def _mtp_storage_label(storage_id: object) -> str:
        try:
            value = int(str(storage_id), 0)
        except (TypeError, ValueError):
            return f"MTP {storage_id or 'unknown'}"
        store_index = (value >> 16) - 1
        if store_index == 0:
            return "APEX-FLASH (QSPI)"
        if store_index == 1:
            return "APEX-SD"
        return f"MTP storage {storage_id}"

    @staticmethod
    def _mtp_listing_failure(text: str) -> str:
        if not text:
            return ""
        interface_claim_failed = "libusb_claim_interface" in text
        no_device = "No Devices have been found" in text
        failure_patterns = [
            "No Devices have been found",
            "LIBMTP PANIC",
            "Unable to open raw device",
            "Unable to initialize device",
            "libusb_claim_interface",
        ]
        if not any(pattern in text for pattern in failure_patterns):
            return ""
        if interface_claim_failed:
            specific = [
                "libmtp found the Teensy, but macOS/libusb refused to claim the MTP interface.",
                "",
                "Most likely another process has the MTP/PTP interface open. If HORIZON is connected over serial, click Disconnect before refreshing Logs; MTP does not need the serial link. Also close OpenMTP, Android File Transfer, Image Capture, Finder import windows, or any other camera/media-transfer app. If it still sticks, unplug the FC, wait a few seconds, plug it back in, and refresh again.",
            ]
        elif no_device:
            specific = [
                "libmtp did not see any MTP device.",
                "",
                "Make sure the FC is powered, the USB cable supports data, and the firmware was built with USB_MTPDISK_SERIAL.",
            ]
        else:
            specific = [
                "libmtp could not open the flight computer as an MTP device.",
            ]
        hints = [
            *specific,
            "",
            "This is a USB/MTP connection failure, not an empty APXLOG folder.",
            "",
            "Raw mtp-files output:",
            text.strip(),
        ]
        return "\n".join(hints)

    def _mtp_capacity(self, progress) -> list[dict]:
        """Read storage capacity/free-space from mtp-detect if available."""
        code, out = self._run_mtp_tool(["mtp-detect"], timeout_s=30)
        if code is None or code != 0:
            progress("libmtp capacity unavailable — mtp-detect did not complete.")
            return []
        rows = self._parse_mtp_detect_capacity(out)
        if not rows:
            progress("libmtp capacity unavailable — mtp-detect did not report storage sizes.")
        return rows

    @staticmethod
    def _parse_intish(value: str) -> int | None:
        value = value.strip()
        try:
            return int(value, 0)
        except ValueError:
            return None

    def _parse_mtp_detect_capacity(self, text: str) -> list[dict]:
        rows: list[dict] = []
        current: dict[str, object] | None = None

        def finish_current():
            if current and (current.get("total") is not None or current.get("free") is not None):
                rows.append(dict(current))

        for line in text.splitlines():
            storage = re.search(r"\bStorage ID:\s*(0x[0-9a-fA-F]+|\d+)", line, re.IGNORECASE)
            if storage:
                finish_current()
                current = {"storage_id": storage.group(1)}
                continue

            if current is None:
                continue

            desc = re.search(r"\b(?:Storage Description|StorageDescription):\s*(.+?)\s*$",
                             line, re.IGNORECASE)
            if desc:
                current["name"] = desc.group(1).strip()

            total = re.search(r"\b(?:Max Capacity|MaxCapacity):\s*(0x[0-9a-fA-F]+|\d+)",
                              line, re.IGNORECASE)
            if total:
                current["total"] = self._parse_intish(total.group(1))

            free = re.search(r"\b(?:Free Space.*?|FreeSpaceInBytes):\s*(0x[0-9a-fA-F]+|\d+)",
                             line, re.IGNORECASE)
            if free:
                current["free"] = self._parse_intish(free.group(1))

        finish_current()
        return rows

    def _pull_entries(self, entries: list[dict], progress) -> dict:
        """Copy only device logs that are not already present locally."""
        archive = _RAW_LOG_ARCHIVE
        _FC_LOG_ARCHIVE.mkdir(parents=True, exist_ok=True)
        copied: list[Path] = []
        skipped: list[str] = []
        errors: list[str] = []
        mode = "mounted volume"
        local_index = self._local_log_index()
        pending: list[dict] = []

        for entry in entries:
            existing = self._find_local_copy_for_entry(entry, local_index)
            if existing is not None:
                skipped.append(str(existing))
            else:
                pending.append(entry)

        if not pending:
            progress("All selected FC logs are already archived locally; nothing to pull.")

        for i, entry in enumerate(pending):
            progress(f"Pulling missing log {entry['name']} ({i + 1}/{len(pending)})…")
            dst = self._entry_archive_path(entry, archive)
            dst.parent.mkdir(parents=True, exist_ok=True)

            if entry["kind"] == "volume":
                src = Path(entry["src"])
                expected_size = entry.get("size")
                if dst.exists():
                    if isinstance(expected_size, int) and dst.stat().st_size == expected_size:
                        skipped.append(str(dst))
                        continue
                    dst = self._dedupe_path(dst)
                shutil.copy2(src, dst)
                copied.append(dst)
                local_index.setdefault((dst.name, dst.stat().st_size), []).append(dst)
                continue

            # MTP entry — mtp-getfile can block for minutes per file.
            mode = "libmtp"
            expected_size = entry.get("size")
            if dst.exists():
                if isinstance(expected_size, int) and dst.stat().st_size == expected_size:
                    skipped.append(str(dst))
                    continue
                if expected_size is None:
                    skipped.append(str(dst))
                    continue
                dst = self._dedupe_path(dst)
            tmp = dst.with_suffix(dst.suffix + ".part")
            if tmp.exists():
                tmp.unlink()
            code, out = self._run_mtp_tool(
                ["mtp-getfile", str(entry.get("id")), str(tmp)], timeout_s=300)
            if code is None or code != 0:
                if tmp.exists():
                    tmp.unlink()
                errors.append(
                    f"{entry['name']}: "
                    f"{out.strip() or f'mtp-getfile exited with status {code}'}")
                continue
            if isinstance(expected_size, int) and tmp.stat().st_size != expected_size:
                got = tmp.stat().st_size
                tmp.unlink()
                errors.append(
                    f"{entry['name']}: downloaded {got} bytes, expected {expected_size}")
                continue
            tmp.replace(dst)
            copied.append(dst)
            local_index.setdefault((dst.name, dst.stat().st_size), []).append(dst)

        if errors:
            raise RuntimeError("Some MTP downloads failed:\n" + "\n".join(errors))
        return {"kind": "pull", "mode": mode, "copied": copied,
                "skipped": skipped, "listing": "",
                "n_requested": len(entries), "n_missing": len(pending)}

    def _trash_local_paths(self, paths: list[Path], progress) -> dict:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        trash_root = _DELETED_LOG_ARCHIVE / stamp
        moved: list[tuple[Path, Path]] = []
        errors: list[str] = []
        for i, src in enumerate(paths):
            progress(f"Moving local log {src.name} ({i + 1}/{len(paths)})…")
            try:
                if not src.exists():
                    errors.append(f"{src}: file no longer exists")
                    continue
                try:
                    rel = src.relative_to(_RAW_LOG_ARCHIVE)
                except ValueError:
                    rel = Path(src.name)
                dst = self._dedupe_path(trash_root / rel)
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src), str(dst))
                moved.append((src, dst))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{src}: {exc}")
        if errors:
            raise RuntimeError("Some local files could not be moved:\n" + "\n".join(errors))
        return {"kind": "delete_local", "moved": moved, "trash_root": trash_root}

    def _delete_device_entries(self, entries: list[dict], progress) -> dict:
        deleted: list[str] = []
        deleted_keys: list[tuple] = []
        rescued: list[Path] = []
        errors: list[str] = []
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        rescue_root = _DELETED_LOG_ARCHIVE / stamp / "from_device"

        for i, entry in enumerate(entries):
            progress(f"Deleting device log {entry['name']} ({i + 1}/{len(entries)})…")
            if entry["kind"] == "mtp":
                code, out = self._run_mtp_tool(
                    ["mtp-delfile", "-n", str(entry.get("id"))], timeout_s=60)
                if code is None or code != 0:
                    errors.append(
                        f"{entry['name']}: "
                        f"{out.strip() or f'mtp-delfile exited with status {code}'}")
                    continue
                deleted.append(str(entry["name"]))
                deleted_keys.append(self._device_entry_key(entry))
                continue

            try:
                src = Path(entry["src"])
                root = Path(entry["root"])
                rel = src.relative_to(root)
                dst = self._dedupe_path(rescue_root / root.parent.name / rel)
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src), str(dst))
                rescued.append(dst)
                deleted.append(str(entry["name"]))
                deleted_keys.append(self._device_entry_key(entry))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{entry['name']}: {exc}")

        if errors:
            raise RuntimeError("Some device files could not be deleted:\n" + "\n".join(errors))
        return {"kind": "delete_device", "deleted": deleted,
                "deleted_keys": deleted_keys, "rescued": rescued}

    def _run_mtp_tool(self, args: list[str], timeout_s: float) -> tuple[int | None, str]:
        exe = shutil.which(args[0])
        if exe is None:
            return None, f"{args[0]} not found. Install libmtp first, e.g. `brew install libmtp`."
        try:
            proc = subprocess.run(
                [exe, *args[1:]],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            out = exc.stdout or ""
            return None, f"{args[0]} timed out after {timeout_s:.0f}s.\n{out}".strip()
        except OSError as exc:
            return None, str(exc)
        return proc.returncode, proc.stdout or ""

    def _parse_mtp_files_output(self, text: str) -> list[dict[str, object]]:
        files: list[dict[str, object]] = []
        current: dict[str, object] | None = None

        def finish_current():
            if current and str(current.get("filename", "")).upper().endswith(".APXLOG"):
                files.append(dict(current))

        for line in text.splitlines():
            file_id = re.search(r"\bFile ID:\s*(\d+)", line, re.IGNORECASE)
            if file_id:
                finish_current()
                current = {"id": int(file_id.group(1))}

            if current is None:
                continue

            filename = re.search(r"\bFilename:\s*(.+?)\s*$", line, re.IGNORECASE)
            if filename:
                current["filename"] = filename.group(1).strip()

            size = re.search(r"\bFile size:?\s*(\d+)", line, re.IGNORECASE)
            if size:
                current["size"] = int(size.group(1))

            parent = re.search(r"\bParent ID:\s*(\d+)", line, re.IGNORECASE)
            if parent:
                current["parent_id"] = parent.group(1)

            storage = re.search(r"\bStorage ID:\s*(0x[0-9a-fA-F]+|\d+)", line, re.IGNORECASE)
            if storage:
                current["storage_id"] = storage.group(1)

        finish_current()
        return files

    def _start_log_job(self, job, busy_msg: str) -> bool:
        if not self._log_ops.start_job(job):
            self._log("[horizon] A log operation is already running")
            return False
        self._set_log_ops_busy(True, busy_msg)
        self._set_log_decode_text(busy_msg)
        return True

    def _format_qspi_flash(self):
        """Erase/reformat the flight computer QSPI flash via guarded serial command."""
        if not self._worker.isRunning():
            self._append_log_decode_text(
                "Not connected over USB serial.\n"
                "Click Connect (this toolbar) to open the flight computer's USB "
                "serial link, then format QSPI.")
            self._log("[horizon] Format QSPI: connect to the flight computer first")
            return

        local_index = self._local_log_index()
        extra_parts: list[str] = []
        if self._device_entries:
            missing_local = [
                entry for entry in self._device_entries
                if self._find_local_copy_for_entry(entry, local_index) is None
            ]
            if missing_local:
                names = "\n".join(
                    f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                    for e in missing_local)
                extra_parts.append(
                    "WARNING: HORIZON cannot find local archive copies for "
                    "these onboard files. Pull them to the laptop before "
                    "formatting unless you are absolutely sure another copy "
                    f"exists:\n{names}")
        else:
            extra_parts.append(
                "WARNING: The Flight Computer file list is empty or stale. "
                "Click Refresh if you want HORIZON to check for local archive "
                "copies before formatting.")

        extra_parts.append(
            "This runs a full low-level erase of the entire QSPI NAND (every "
            "block, not just the filesystem header), so it also clears any "
            "block-level corruption. It wipes all APXLOG files and boot/flight "
            "counters, then starts a fresh log session. The erase blocks the "
            "flight computer for several seconds.")

        if not self._confirm_dangerous_delete(
            "Format QSPI flash",
            "Full-erase and reformat the flight computer's QSPI flash?",
            "\n\n".join(extra_parts),
            phrase=_FORMAT_QSPI_CONFIRM_PHRASE):
            self._set_log_decode_text("QSPI format canceled.")
            return

        self._worker.send_bytes(b"FORMAT_QSPI_ERASE_ALL\n")
        self._append_log_decode_text(
            "Sent FORMAT_QSPI_ERASE_ALL.\n"
            "Watch the log panel for the firmware's proof line:\n"
            "  '#INFO: QSPI LittleFS erased — fresh BOOT_00001 ...'\n"
            "Then click Refresh. APEX-FLASH should contain only the fresh "
            "session; APEX-SD is not erased and may still list older logs.")
        self._log("[horizon] Sent QSPI low-level format command")

    def _refresh_device_files(self):
        """List .APXLOG files present on the flight computer — in the worker
        thread (mtp-files alone can block for tens of seconds)."""

        # The table is only the latest mtp-files snapshot. Clear the previous
        # snapshot while refreshing; never substitute laptop archive entries.
        self._device_entries = []
        self._visible_device_entries = []
        self.device_table.clearContents()
        self.device_table.setRowCount(0)

        def job(progress):
            mode, entries, listing, capacity = self._list_device_entries(progress)
            return {"kind": "device_list", "mode": mode,
                    "entries": entries, "listing": listing,
                    "capacity": capacity}

        self._start_log_job(job, "Listing files on flight computer…")

    def _populate_device_table(self, entries: list[dict]):
        self._device_entries = self._annotate_device_entries(entries)
        self._render_device_table()

    def _render_device_table(self, *_args):
        mode = (self.device_filter_combo.currentData()
                if hasattr(self, "device_filter_combo") else "all")
        if mode == "qspi":
            visible = [entry for entry in self._device_entries
                       if entry.get("source") == "APEX-FLASH (QSPI)"]
        elif mode == "sd":
            visible = [entry for entry in self._device_entries
                       if entry.get("source") == "APEX-SD"]
        else:
            visible = list(self._device_entries)

        self._visible_device_entries = visible
        self.device_table.clearContents()
        self.device_table.setRowCount(len(visible))
        for i, entry in enumerate(visible):
            status = str(entry.get("local_status", "missing"))
            cells = [str(entry["name"]), self._fmt_size(entry.get("size")),
                     status, str(entry.get("source", ""))]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == 1:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if col == 2:
                    item.setForeground(QColor(GOOD if status == "archived" else AMBER))
                    tooltip = entry.get("local_path") or entry.get("archive_path") or ""
                    item.setToolTip(str(tooltip))
                self.device_table.setItem(i, col, item)

    def _set_device_capacity(self, rows: list[dict] | None):
        rows = rows or []
        if not rows:
            self.device_capacity_label.setText(
                "Capacity: unavailable from device (QSPI nominal: 128 MiB / 1 Gbit)")
            return
        parts = []
        for row in rows:
            total = row.get("total")
            free = row.get("free")
            used = (total - free
                    if isinstance(total, int) and isinstance(free, int)
                    else None)
            label = str(row.get("name") or row.get("storage_id") or "storage")
            sid = row.get("storage_id")
            if sid and sid != label:
                label = f"{label} ({sid})"
            if isinstance(total, int) and isinstance(free, int):
                parts.append(
                    f"{label}: {self._fmt_size(used)} used / {self._fmt_size(total)} total "
                    f"({self._fmt_size(free)} free)")
            elif isinstance(total, int):
                parts.append(f"{label}: {self._fmt_size(total)} total")
            elif isinstance(free, int):
                parts.append(f"{label}: {self._fmt_size(free)} free")
        self.device_capacity_label.setText("Capacity: " + "   |   ".join(parts))

    def _selected_device_entries(self) -> list[dict]:
        rows = sorted({idx.row() for idx in self.device_table.selectedIndexes()})
        return [self._visible_device_entries[r] for r in rows
                if 0 <= r < len(self._visible_device_entries)]

    def _start_pull_job(self, entries: list[dict], export_after: bool):
        """Pull missing copies of the given device entries into the archive."""
        self._log_export_after_pull = export_after
        snapshot = [dict(e) for e in entries]
        missing = self._missing_device_entries(snapshot)
        if not missing:
            local_paths = [self._find_local_copy_for_entry(entry) for entry in snapshot]
            focus_paths = [path for path in local_paths if path is not None]
            self._set_log_decode_text(
                f"All {len(snapshot)} selected FC log file(s) are already archived locally.\n"
                f"Laptop archive: {_RAW_LOG_ARCHIVE}")
            self._populate_device_table(self._device_entries)
            self._refresh_local_log_choices(focus_paths=focus_paths or None)
            if export_after:
                self._log_export_after_pull = False
                self._export_logs(summary_only=False, all_files=False)
            return

        def job(progress):
            return self._pull_entries(snapshot, progress)

        self._start_log_job(job, f"Pulling {len(missing)} missing log(s) from flight computer…")

    def _pull_selected_device_files(self):
        entries = self._selected_device_entries()
        if not entries:
            self._set_log_decode_text(
                "No device files selected — Refresh the Flight Computer list "
                "and select rows first.")
            return
        self._start_pull_job(entries, export_after=False)

    def _pull_all_device_files(self, export_after: bool = False):
        if self._device_entries:
            self._start_pull_job(self._visible_device_entries, export_after)
            return
        # No Refresh yet — discover and pull everything in one job.
        self._log_export_after_pull = export_after

        def job(progress):
            mode, entries, listing, capacity = self._list_device_entries(progress)
            if not entries:
                return {"kind": "pull", "mode": mode, "copied": [],
                        "skipped": [], "listing": listing, "n_requested": 0,
                        "n_missing": 0, "capacity": capacity, "entries": []}
            result = self._pull_entries(entries, progress)
            result["mode"] = mode
            result["listing"] = listing
            result["capacity"] = capacity
            result["entries"] = entries
            return result

        self._start_log_job(job, "Pulling logs from flight computer…")

    def _delete_selected_device_files(self):
        entries = self._selected_device_entries()
        if not entries:
            self._set_log_decode_text(
                "No device files selected — Refresh the Flight Computer list "
                "and select rows first.")
            return

        local_index = self._local_log_index()
        missing_local = [
            entry for entry in entries
            if self._find_local_copy_for_entry(entry, local_index) is None
        ]
        extra = ""
        if missing_local:
            names = "\n".join(f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                              for e in missing_local)
            extra = (
                "WARNING: HORIZON cannot find local archive copies for these "
                "selected device files. Pull them to the laptop before deleting "
                "unless you are absolutely sure another copy exists:\n"
                f"{names}")

        if not self._confirm_dangerous_delete(
            "Delete logs from flight computer",
            f"Delete {len(entries)} selected APXLOG file(s) from the flight computer?",
            extra):
            self._set_log_decode_text("Device delete canceled.")
            return

        snapshot = [dict(e) for e in entries]

        def job(progress):
            return self._delete_device_entries(snapshot, progress)

        self._start_log_job(job, "Deleting logs from flight computer…")

    def _delete_selected_local_logs(self):
        paths = self._selected_local_log_paths()
        if not paths:
            self._set_log_decode_text("No local archive files selected.")
            return
        if not self._device_entries:
            self._set_log_decode_text(
                "Refresh the Flight Computer file list before deleting local logs. "
                "HORIZON needs the current device list to check whether another "
                "copy is still onboard.")
            return

        missing_on_device = [
            path for path in paths
            if not any(self._device_entry_matches_local_path(entry, path)
                       for entry in self._device_entries)
        ]
        extra = ""
        if missing_on_device:
            names = "\n".join(f"  {path.name}" for path in missing_on_device)
            extra = (
                "WARNING: These local files are not currently listed on the "
                "flight computer. Moving them may leave no onboard copy:\n"
                f"{names}")

        if not self._confirm_dangerous_delete(
            "Delete local archive logs",
            f"Move {len(paths)} selected local APXLOG file(s) to the recently deleted folder?",
            extra):
            self._set_log_decode_text("Local delete canceled.")
            return

        snapshot = [Path(p) for p in paths]

        def job(progress):
            return self._trash_local_paths(snapshot, progress)

        self._start_log_job(job, "Moving local logs to recently deleted…")

    def _export_logs(self, summary_only: bool = False, all_files: bool = False):
        # Gather widget state on the UI thread; the job itself must not
        # touch Qt objects.
        used_default = False
        if all_files:
            raw_inputs: list[Path] = self._local_log_paths()
        else:
            raw_inputs = self._selected_local_log_paths()
            if not raw_inputs:
                raw_inputs = self._local_log_paths()
                used_default = True
        if not raw_inputs:
            self._set_log_decode_text(
                "The laptop archive is empty.\n"
                "Pull from the Flight Computer first, or Add external… files.")
            return
        out_dir = Path(self.log_output_field.text().strip()
                       or (_SIM_ROOT / "output" / "log_exports"))

        def job(progress):
            from apex_sim.logs.decoder import (
                decode_files, export_logs, iter_log_paths)
            paths = list(iter_log_paths(raw_inputs))
            if not paths:
                raise RuntimeError("No .APXLOG files found in the selected input.")
            progress(f"Decoding {len(paths)} file(s)…")
            records, stats = decode_files(paths)
            rows = flight_summaries(records)
            written = []
            if not summary_only:
                progress("Writing CSV export…")
                written = export_logs(paths, out_dir, include_ground=True)
            return {"kind": "export", "stats": stats, "rows": rows,
                    "written": written, "out_dir": out_dir,
                    "n_paths": len(paths), "summary_only": summary_only,
                    "used_default": used_default,
                    "boots": sorted({r.boot_id for r in records}),
                    "flights": sorted({r.flight_id for r in records
                                       if r.flight_id})}

        self._start_log_job(
            job, "Decoding logs…" if summary_only else "Decoding + exporting…")

    def _on_log_job_failed(self, msg: str):
        self._set_log_ops_busy(False)
        self._log_export_after_pull = False
        self._append_log_decode_text(f"\nFAILED: {msg}")
        self._log(f"[horizon] Log operation failed: {msg}")

    def _on_log_job_done(self, result: dict):
        self._set_log_ops_busy(False)
        if result["kind"] == "device_list":
            entries = result["entries"]
            self._populate_device_table(entries)
            self._set_device_capacity(result.get("capacity"))
            missing = [e for e in self._device_entries if e.get("local_status") != "archived"]
            qspi_count = sum(e.get("source") == "APEX-FLASH (QSPI)"
                             for e in self._device_entries)
            sd_count = sum(e.get("source") == "APEX-SD"
                           for e in self._device_entries)
            lines = [f"Found {len(entries)} .APXLOG file(s) on the device "
                     f"via {result['mode']}.",
                     f"Device storage: QSPI={qspi_count}, SD={sd_count}",
                     f"Local archive: {_RAW_LOG_ARCHIVE}",
                     f"Already archived: {len(entries) - len(missing)}; missing: {len(missing)}"]
            if missing:
                lines.append("Missing on laptop:")
                lines.extend(f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                             for e in missing[:20])
                if len(missing) > 20:
                    lines.append(f"  … {len(missing) - 20} more")
            if not entries and result["mode"] == "libmtp":
                lines += ["", "libmtp connected but listed no .APXLOG files.",
                          "Raw mtp-files output:", result["listing"].strip()]
            self._append_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Device listing: {len(entries)} log file(s), "
                      f"{len(missing)} missing locally via {result['mode']}")
            return

        if result["kind"] == "pull":
            copied = result["copied"]
            if "capacity" in result:
                self._set_device_capacity(result.get("capacity"))
            if result.get("entries"):
                self._populate_device_table(result["entries"])
            else:
                self._populate_device_table(self._device_entries)
            lines = [
                f"Pulled logs via {result['mode']}: "
                f"copied {len(copied)} missing file(s), "
                f"skipped {len(result['skipped'])} already archived",
                f"Requested: {result.get('n_requested', 0)}; missing before pull: {result.get('n_missing', len(copied))}",
                f"Laptop archive: {_RAW_LOG_ARCHIVE}",
            ]
            if copied:
                lines.append("Copied:")
                lines.extend(f"  {p}" for p in copied)
            elif result["mode"] == "libmtp" and not result["skipped"]:
                lines += ["", "libmtp connected but listed no .APXLOG files.",
                          "Raw mtp-files output:", result["listing"].strip()]
            else:
                lines.append("No transfers needed; every requested FC log already has a local copy.")
            self._set_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Pulled {len(copied)} missing log file(s), "
                      f"skipped {len(result['skipped'])}")
            focus_paths = copied or [Path(p) for p in result["skipped"]]
            self._refresh_local_log_choices(focus_paths=focus_paths or None)
            if self._log_export_after_pull:
                self._log_export_after_pull = False
                self._export_logs(summary_only=False, all_files=False)
            return

        if result["kind"] == "delete_local":
            moved = result["moved"]
            lines = [
                f"Moved {len(moved)} local APXLOG file(s) to recently deleted.",
                f"Recently deleted folder: {result['trash_root']}",
            ]
            if moved:
                lines.append("Moved:")
                lines.extend(f"  {src}  ->  {dst}" for src, dst in moved)
            self._set_log_decode_text("\n".join(lines))
            self._refresh_local_log_choices(select_all=False)
            self._log(f"[horizon] Moved {len(moved)} local log file(s) to recently deleted")
            return

        if result["kind"] == "delete_device":
            deleted = result["deleted"]
            deleted_keys = set(tuple(key) for key in result.get("deleted_keys", []))
            self._device_entries = [
                entry for entry in self._device_entries
                if self._device_entry_key(entry) not in deleted_keys
            ]
            self._populate_device_table(self._device_entries)
            lines = [f"Deleted {len(deleted)} APXLOG file(s) from the flight computer."]
            if result["rescued"]:
                lines.extend([
                    "",
                    "Mounted-volume files were moved into a local rescue folder:",
                    *(f"  {p}" for p in result["rescued"]),
                ])
            if deleted:
                lines.append("")
                lines.append("Deleted:")
                lines.extend(f"  {name}" for name in deleted)
            self._set_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Deleted {len(deleted)} device log file(s)")
            return

        # Export / summary result
        stats = result["stats"]
        self._populate_flights_table(result["rows"])
        lines = [
            f"Decoded {stats.records} records from {result['n_paths']} file(s)"
            + ("  [all local archive logs]" if result["used_default"] else ""),
            f"boot_ids={result['boots']}  flight_ids={result['flights']}",
            f"bad_crc={stats.bad_crc}  truncated={stats.truncated}  "
            f"resync_bytes={stats.resync_bytes}",
        ]
        if not result["summary_only"]:
            self._last_export_dir = result["out_dir"]
            written = result["written"]
            lines.append("")
            if written:
                lines.append(f"Wrote {len(written)} file(s) to {result['out_dir']}:")
                lines.extend(f"  {p}" for p in written)
            else:
                lines.append("No CSV files were written.")
            self._log(f"[horizon] Exported {len(written)} log CSV file(s)")
        else:
            self._log(f"[horizon] Log summary: {stats.records} records, "
                      f"flights={result['flights']}")
        self._set_log_decode_text("\n".join(lines))

    def _set_disconnected(self):
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("")

    def _on_worker_finished(self):
        if (not self._worker.isRunning() and not self._radio_worker.isRunning()
                and not self._hil_worker.isRunning()):
            self._set_disconnected()

    def _send_command(self):
        text = self.cmd_input.text().strip()
        if not text:
            return
        if self._source_mode() == "radio":
            self._log("[horizon] Radio RX is receive-only; use USB serial to send commands")
            self.cmd_input.clear()
            return
        if self._source_mode() == "logs":
            self._log("[horizon] Logs tab has no serial command target")
            self.cmd_input.clear()
            return
        self._worker.send_bytes(text.encode() + b"\n")
        self._log(f"[TX] {text}")
        self.cmd_input.remember(text)
        self.cmd_input.clear()

    # ── Data routing ──────────────────────────────────────────────────────────

    def _on_lines(self, lines: list):
        """Batched protocol lines from the HIL worker (one batch per tick)."""
        at = time.monotonic()
        for line in lines:
            self._on_line(line, at)

    def _on_line(self, line: str, at: float = None):
        now = time.monotonic() if at is None else at
        if self._t_start is None:
            self._t_start = now
        t = now - self._t_start

        if line.startswith(">"):
            # Numeric value
            try:
                key, val_str = line[1:].split(":", 1)
                value = float(val_str)
                mode = self._source_mode()
                key_map = (self._key_to_group_radio if mode == "radio"
                           else self._key_to_group_hil if mode == "hil"
                           else self._key_to_group_serial)
                group = key_map.get(key)
                if group is None:
                    group = self._get_or_create_overflow()
                    key_map[key] = group
                    group.add_key(key)
                group.push(key, t, value)
                self.state_panel.update_value(key, value)
            except (ValueError, IndexError):
                self._log(line)

        elif line.startswith("!"):
            # State / flag
            try:
                key, value = line[1:].split(":", 1)
                key = key.strip()
                value = value.strip()
                if key == "hil_event" and self._source_mode() == "hil":
                    self._add_hil_event(value, t)
                else:
                    self.state_panel.update_state(key, value)
            except (ValueError, IndexError):
                self._log(line)

        else:
            # Log line (# prefix or raw text)
            self._log(line.lstrip("#"))

    def _get_or_create_overflow(self) -> PlotGroupWidget:
        if self._overflow_group is None:
            colors = ["#ff55ff", "#55ffff", "#ffff55", "#ff8855", "#55ff88"]
            self._overflow_group = PlotGroupWidget("Other", [], colors)
            self._plot_layout.insertWidget(self._plot_layout.count() - 1, self._overflow_group)
            self._plot_groups.append(self._overflow_group)
        return self._overflow_group

    def _add_hil_event(self, event: str, t: float):
        style = HIL_EVENT_STYLES.get(event)
        if style is None:
            return
        label, color = style
        for group in self._hil_groups:
            group.add_event(t, label, color)

    # Log level → (color, bold)
    _LOG_STYLES = {
        "ERROR": ("#ff4444", True),
        "WARN":  ("#ffaa00", False),
        "INFO":  ("#8888cc", False),
        "DEBUG": ("#555577", False),
    }

    def _log(self, text: str):
        text = text.strip()
        if not text:
            return

        color, bold = "#aaaaaa", False
        prefix = ""

        # Firmware plot-mode prefix: #ERROR: / #WARN: / #INFO:
        for level, (c, b) in self._LOG_STYLES.items():
            if text.startswith(level + ":"):
                color, bold = c, b
                text = text[len(level) + 1:].lstrip()
                ts = time.strftime("%H:%M:%S")
                prefix = (
                    f'<span style="color:#444466;">{ts}</span> '
                    f'<span style="color:{c};font-weight:{"bold" if b else "normal"};">[{level}]</span> '
                )
                break
        else:
            # [horizon] and [TX] internal messages
            if text.startswith("[horizon]"):
                color = "#446688"
            elif text.startswith("[TX]"):
                color = "#44ffaa"

        weight = "bold" if bold else "normal"
        body = f'<span style="color:{color};font-weight:{weight};">{html.escape(text)}</span>'
        self.log_view.append(prefix + body)
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Plot refresh ──────────────────────────────────────────────────────────

    def _refresh_plots(self):
        t_now = None if self._t_start is None else time.monotonic() - self._t_start
        for group in self._plot_groups:
            # Skip raster work for groups on another page (isVisible) or
            # scrolled out of the viewport (empty visibleRegion). push() is
            # unaffected — data accumulates and the group repaints fully when
            # it scrolls back into view.
            if not group.isVisible() or group.visibleRegion().isEmpty():
                group.mark_offscreen()
                continue
            group.refresh(self._window_s, t_now)
        self.state_panel.flush_values()

    # ── Status ────────────────────────────────────────────────────────────────

    def _on_reconnect(self):
        """Teensy reset and re-enumerated — clear display so boot messages are visible."""
        self._clear_data()
        self._log("[horizon] Device reconnected — display cleared")

    def _on_status(self, msg: str, is_error: bool):
        color = "#dd4444" if is_error else "#44dd88"
        self.status_label.setText(msg)
        self.status_label.setStyleSheet(f"color:{color};")
        self._log(f"[horizon] {msg}")

    # ── Clipboard export ──────────────────────────────────────────────────────

    def _copy_to_clipboard(self):
        window_s = self.state_panel.copy_window_spin.value()
        lines = [
            f"# HORIZON — last {window_s}s",
            f"# {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "",
        ]

        for group in self._plot_groups:
            keys_with_data = [k for k in group.keys if group._t[k]]
            if not keys_with_data:
                continue

            # Find the global t_min across all keys in this group
            t_maxes = [np.array(group._t[k])[-1] for k in keys_with_data]
            t_max   = max(t_maxes)
            t_min   = t_max - window_s

            # Filter each key to the window
            filtered: dict[str, tuple] = {}
            for k in keys_with_data:
                t_arr = np.array(group._t[k])
                y_arr = np.array(group._y[k])
                mask  = t_arr >= t_min
                filtered[k] = (t_arr[mask], y_arr[mask])

            if not any(len(filtered[k][0]) for k in keys_with_data):
                continue

            lines.append(f"# {group.title()}")
            lines.append("time_s," + ",".join(keys_with_data))

            # Use the longest key as the time axis; zip fills with "" for shorter series
            ref_key = max(keys_with_data, key=lambda k: len(filtered[k][0]))
            ref_t   = filtered[ref_key][0]

            for i, t_val in enumerate(ref_t):
                row = [f"{t_val:.4f}"]
                for k in keys_with_data:
                    t_k, y_k = filtered[k]
                    row.append(f"{y_k[i]:.6g}" if i < len(y_k) else "")
                lines.append(",".join(row))

            lines.append("")

        text = "\n".join(lines)
        QApplication.clipboard().setText(text)
        n = sum(1 for l in lines if l and not l.startswith("#"))
        self._log(f"[horizon] Copied {n} data rows ({window_s}s window) to clipboard")

    # ── Util ──────────────────────────────────────────────────────────────────

    def _clear_data(self):
        self._t_start = None
        for group in self._plot_groups:
            group.clear()
        self.state_panel.reset_flight()
        self.log_view.clear()   # QTextEdit.clear()

    def _apply_dark_theme(self):
        self.setStyleSheet(f"""
            /* ── Base ─────────────────────────────────────────────────────── */
            QMainWindow, QWidget    {{ background: {BG}; color: {TEXT}; }}
            QLabel                  {{ color: {TEXT}; background: transparent; }}
            QScrollArea             {{ border: none; }}
            QSplitter::handle       {{ background: {BORDER}; width: 2px; }}
            QSplitter::handle:hover {{ background: {ACCENT}; }}
            QToolTip                {{ background: {SURFACE}; color: {TEXT};
                                       border: 1px solid {BORDER_HI}; padding: 3px 6px; }}

            /* ── Page tabs ────────────────────────────────────────────────── */
            QFrame#tabstrip         {{ background: {BG};
                                       border-bottom: 1px solid {BORDER}; }}
            QTabBar                 {{ background: transparent; }}
            QTabBar::tab            {{ background: transparent; color: {TEXT_DIM};
                                       padding: 7px 18px 5px 18px;
                                       border: none;
                                       border-bottom: 2px solid transparent;
                                       font-weight: bold; }}
            QTabBar::tab:hover      {{ color: {TEXT}; }}
            QTabBar::tab:selected   {{ color: {ACCENT};
                                       border-bottom: 2px solid {ACCENT}; }}
            QTabBar::tab:disabled   {{ color: {BORDER_HI}; }}

            /* ── Toolbar ──────────────────────────────────────────────────── */
            QFrame#toolbar          {{ background: #111122;
                                       border-bottom: 1px solid {BORDER}; }}
            QFrame#toolbar QLabel   {{ color: #9090b8; background: transparent; }}

            /* Toolbar controls: inset look — darker than toolbar surface */
            QFrame#toolbar QComboBox,
            QFrame#toolbar QSpinBox,
            QFrame#toolbar QDoubleSpinBox,
            QFrame#toolbar QLineEdit {{ background: {INSET}; border: 1px solid #2e2e50;
                                        color: {TEXT}; padding: 2px 6px; border-radius: 3px; }}
            QFrame#toolbar QPushButton
                                    {{ background: {INSET}; border: 1px solid #2e2e50;
                                       color: #aaaacc; padding: 3px 10px; border-radius: 3px; }}
            QFrame#toolbar QPushButton:hover
                                    {{ background: #14142a; border-color: {ACCENT};
                                       color: #ddddff; }}

            /* ── Panel controls (outside toolbar) ────────────────────────── */
            QGroupBox               {{ border: 1px solid {BORDER}; border-radius: 6px;
                                       margin-top: 7px; padding-top: 7px;
                                       color: {TEXT_DIM}; }}
            QGroupBox::title        {{ subcontrol-origin: margin; left: 10px;
                                       padding: 0 4px; color: {ACCENT}; }}

            QComboBox, QSpinBox,
            QDoubleSpinBox, QLineEdit, QListWidget
                                    {{ background: {INSET}; border: 1px solid {BORDER_HI};
                                       color: {TEXT}; padding: 2px 6px; border-radius: 3px; }}
            QComboBox:focus, QSpinBox:focus,
            QDoubleSpinBox:focus, QLineEdit:focus, QListWidget:focus
                                    {{ border-color: {ACCENT}; }}
            QListWidget::item       {{ padding: 3px 6px; }}
            QListWidget::item:selected
                                    {{ background: #2a2a50; color: #ffffff; }}
            QListWidget::item:hover {{ background: #242444; }}
            QPushButton             {{ background: #1e1e3a; border: 1px solid {BORDER_HI};
                                       color: {TEXT}; padding: 3px 10px; border-radius: 3px; }}
            QPushButton:hover       {{ background: #2a2a4a; border-color: {ACCENT}; }}
            QPushButton:pressed     {{ background: #16162e; }}

            /* ── ComboBox popup ──────────────────────────────────────────── */
            QComboBox QAbstractItemView {{
                                       background: {SURFACE};
                                       border: 1px solid {BORDER_HI};
                                       color: {TEXT};
                                       selection-background-color: #2a2a50;
                                       selection-color: #ffffff;
                                       outline: none; }}

            /* ── Scrollbars ───────────────────────────────────────────────── */
            QScrollBar:vertical     {{ background: {BG}; width: 8px;
                                       margin: 0; border: none; }}
            QScrollBar::handle:vertical
                                    {{ background: #2e2e50; border-radius: 4px;
                                       min-height: 24px; }}
            QScrollBar::handle:vertical:hover
                                    {{ background: #44447a; }}
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical  {{ height: 0; }}
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical  {{ background: none; }}

            QScrollBar:horizontal   {{ background: {BG}; height: 8px;
                                       margin: 0; border: none; }}
            QScrollBar::handle:horizontal
                                    {{ background: #2e2e50; border-radius: 4px;
                                       min-width: 24px; }}
            QScrollBar::handle:horizontal:hover
                                    {{ background: #44447a; }}
            QScrollBar::add-line:horizontal,
            QScrollBar::sub-line:horizontal {{ width: 0; }}
            QScrollBar::add-page:horizontal,
            QScrollBar::sub-page:horizontal {{ background: none; }}
        """)
        pg.setConfigOption("background", BG)
        pg.setConfigOption("foreground", "#888899")

    def closeEvent(self, event):
        self._worker.stop()
        self._radio_worker.stop()
        self._hil_worker.stop()
        self._log_ops.wait(1000)   # jobs are short except MTP; don't hang exit
        event.accept()


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app._horizon_style = HorizonStyle(app.style())
    app.setStyle(app._horizon_style)
    app.setApplicationName("HORIZON — Apex Ground")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
