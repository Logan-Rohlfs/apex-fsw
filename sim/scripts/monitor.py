#!/usr/bin/env python3
"""
Apex Flight Computer — Serial Monitor

Unified interface for laptop-connected Teensy (APEX_MONITOR build).
Accepts three line formats from firmware:
  >key:value    numeric — routed to live plots and values table
  !key:value    state   — routed to state panel (phase, health flags, etc.)
  #LEVEL: msg   log     — shown in the log panel with color coding

Sends newline-terminated ASCII commands to the Teensy via the command
input at the bottom of the log panel. Commands: ARM, DISARM (more once
the state machine is implemented).

Run: python scripts/monitor.py
"""

import sys
import time
import shutil
import subprocess
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSplitter, QScrollArea,
    QFrame, QPlainTextEdit, QTextEdit, QSizePolicy, QGridLayout, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QLineEdit, QCompleter,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex
from PyQt5.QtGui import QFont, QColor

from radio_ook_rx import (
    DEFAULT_BIT_MS,
    DEFAULT_FREQ_HZ,
    DEFAULT_PAYLOAD_LEN,
    DEFAULT_SAMPLE_RATE_HZ,
    envelope_bins,
    find_best_frame,
    is_valid_frame,
    load_u8_iq,
)

# ─── Plot group definitions ───────────────────────────────────────────────────
# (group_title, [keys...], [hex_colors...])
# Keys matched against incoming >key:value lines. Unmatched keys go to "Other".

PLOT_GROUPS = [
    ("Fusion",    ["alt_agl", "velocity", "pred_apogee"],  ["#00d4ff", "#ff6b35", "#a8ff3e"]),
    ("IMU Accel", ["accel_x", "accel_y", "accel_z"],       ["#ff4444", "#44ff44", "#4488ff"]),
    ("IMU Gyro",  ["gyro_x",  "gyro_y",  "gyro_z"],        ["#ff8844", "#88ff44", "#4488ff"]),
    ("High-G",    ["highg_x", "highg_y", "highg_z"],       ["#ff44aa", "#44ffaa", "#aa44ff"]),
    ("Baro Alt",  ["baro_alt"],                               ["#ffdd44"]),
    ("Baro Temp", ["baro_temp"],                             ["#ff9922"]),
    ("Mag",       ["mag_x",   "mag_y",   "mag_z"],          ["#ff9933", "#33ff99", "#9933ff"]),
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

KNOWN_COMMANDS = [
    "ARM",
    "DISARM",
    "RADIO_DATA_TEST",
    "RADIO_MARKER",
    "RADIO_MARKER_441",
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
    status_changed = pyqtSignal(str, bool)

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
        self._last_payload: bytes | None = None
        self._last_emit_s = 0.0

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
            "-",
        ]
        if self._gain.lower() != "auto":
            cmd[1:1] = ["-g", self._gain]

        self._running = True
        self._packet_count = 0
        self._last_payload = None
        self._last_emit_s = 0.0

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
        self.line_received.emit("#INFO: radio rx waiting for RADIO_DATA_TEST frame")

        raw = bytearray()
        max_bytes = int(self._sample_rate * 2 * 14)  # unsigned 8-bit IQ, ~14 s rolling window
        min_bytes = int(self._sample_rate * 2 * 11)  # one 10 s OOK frame plus margin
        read_size = 131072
        last_decode_s = 0.0

        try:
            while self._running:
                if self._proc.stdout is None:
                    break
                chunk = self._proc.stdout.read(read_size)
                if not chunk:
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.05)
                    continue

                raw.extend(chunk)
                if len(raw) > max_bytes:
                    del raw[:len(raw) - max_bytes]

                now = time.monotonic()
                if len(raw) >= min_bytes and now - last_decode_s >= 0.75:
                    last_decode_s = now
                    self._try_decode(bytes(raw), now)
        finally:
            self._terminate_proc()
            self._running = False

    def _try_decode(self, raw: bytes, now_s: float):
        try:
            samples = load_u8_iq(raw)
            values = envelope_bins(samples, self._sample_rate)
            samples_per_bit = max(1, int(round(DEFAULT_BIT_MS)))
            result = find_best_frame(values, samples_per_bit, DEFAULT_PAYLOAD_LEN)
        except Exception as exc:
            self.line_received.emit(f"#WARN: radio rx decode error: {exc}")
            return

        if result is None:
            return

        if not is_valid_frame(result):
            return

        if result.payload == self._last_payload and now_s - self._last_emit_s < 8.0:
            return

        self._last_payload = result.payload
        self._last_emit_s = now_s
        self._packet_count += 1

        try:
            text = result.payload.decode("ascii")
        except UnicodeDecodeError:
            text = result.payload.hex(" ")

        level = "INFO" if result.checksum_ok else "WARN"
        status = "OK" if result.checksum_ok else "BAD"
        self.line_received.emit(
            f"#{level}: radio packet #{self._packet_count} payload=\"{text}\" "
            f"checksum={status} preamble_errors={result.errors}"
        )
        self.line_received.emit(">radio_payload_seen:1")
        self.line_received.emit(f">radio_packet_count:{self._packet_count}")
        self.line_received.emit(f"!radio_rx:{1 if result.checksum_ok else -1}")

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


# ─── Plot group widget ────────────────────────────────────────────────────────

class PlotGroupWidget(QGroupBox):
    def __init__(self, title: str, keys: list, colors: list, window_s: int = DEFAULT_WINDOW_S):
        super().__init__(title)
        self.keys    = keys
        self.colors  = colors
        self.window_s = window_s

        self._t   = {k: deque(maxlen=MAX_POINTS) for k in keys}
        self._y   = {k: deque(maxlen=MAX_POINTS) for k in keys}
        self._t0  = None   # first timestamp seen

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.plot = PlotWidget(background="#1a1a2e")
        self.plot.setMinimumHeight(160)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.getAxis("bottom").setLabel("t (s)")
        self.plot.addLegend(offset=(-10, 10))

        self.curves = {}
        for i, key in enumerate(keys):
            color = colors[i % len(colors)]
            self.curves[key] = self.plot.plot(
                pen=pg.mkPen(color, width=1.5),
                name=key,
            )

        layout.addWidget(self.plot)

        # Enable key if data arrives (dim unused keys in legend)
        self._seen = set()

    def push(self, key: str, t: float, value: float):
        if key not in self._t:
            return
        if self._t0 is None:
            self._t0 = t
        self._t[key].append(t - self._t0)
        self._y[key].append(value)
        self._seen.add(key)

    def refresh(self, window_s: int):
        for key in self.keys:
            if not self._t[key]:
                continue
            t_arr = np.array(self._t[key])
            y_arr = np.array(self._y[key])
            t_max = t_arr[-1]
            t_min = t_max - window_s
            mask  = t_arr >= t_min
            self.curves[key].setData(t_arr[mask], y_arr[mask])
        # Roll the x-axis view
        if self._t0 is not None:
            elapsed = time.monotonic()
            # just let pyqtgraph auto-range; user can lock axes manually
            self.plot.enableAutoRange(axis="x")

    def add_key(self, key: str, color: str = "#ffffff"):
        """Dynamically add a key not in the original definition."""
        if key in self._t:
            return
        self.keys.append(key)
        self.colors.append(color)
        self._t[key] = deque(maxlen=MAX_POINTS)
        self._y[key] = deque(maxlen=MAX_POINTS)
        self.curves[key] = self.plot.plot(
            pen=pg.mkPen(color, width=1.5),
            name=key,
        )

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
        self.phase_label.setFont(QFont("Courier", 22, QFont.Bold))
        self.phase_label.setFixedHeight(54)
        self.phase_label.setStyleSheet(
            f"background:{PHASE_COLORS['IDLE']}; color:#ffffff; border-radius:6px;"
        )
        layout.addWidget(self.phase_label)

        # Sensor health row
        health_box = QGroupBox("Sensors")
        health_layout = QHBoxLayout(health_box)
        health_layout.setContentsMargins(6, 4, 6, 4)
        self.sensor_dots: dict[str, QLabel] = {}
        for name in SENSOR_BITS:
            dot = QLabel(name)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(QFont("Courier", 9, QFont.Bold))
            dot.setFixedSize(52, 22)
            dot.setStyleSheet("background:#444; color:#888; border-radius:4px;")
            self.sensor_dots[name] = dot
            health_layout.addWidget(dot)
        layout.addWidget(health_box)

        # Radio status
        radio_box = QGroupBox("Radio")
        radio_layout = QHBoxLayout(radio_box)
        radio_layout.setContentsMargins(6, 4, 6, 4)
        self.radio_label = QLabel("OFFLINE")
        self.radio_label.setAlignment(Qt.AlignCenter)
        self.radio_label.setFont(QFont("Courier", 9, QFont.Bold))
        self.radio_label.setFixedHeight(20)
        self.radio_label.setStyleSheet("background:#aa2222; color:#fff; border-radius:3px; padding:0 4px;")
        radio_layout.addWidget(self.radio_label)
        layout.addWidget(radio_box)

        # GPS status row
        gps_box = QGroupBox("GPS")
        gps_layout = QHBoxLayout(gps_box)
        gps_layout.setContentsMargins(6, 4, 6, 4)

        self.gps_fix_label = QLabel("NO FIX")
        self.gps_fix_label.setFont(QFont("Courier", 9, QFont.Bold))
        self.gps_fix_label.setAlignment(Qt.AlignCenter)
        self.gps_fix_label.setFixedHeight(20)
        self.gps_fix_label.setStyleSheet("background:#444; color:#888; border-radius:3px; padding:0 4px;")

        self.gps_sats_label = QLabel("0 sats")
        self.gps_sats_label.setFont(QFont("Courier", 9))
        self.gps_sats_label.setStyleSheet("color:#888;")

        self.gps_utc_label = QLabel("—")
        self.gps_utc_label.setFont(QFont("Courier", 8))
        self.gps_utc_label.setStyleSheet("color:#888;")

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
        values_outer.addWidget(self._values_grid)
        layout.addWidget(values_box)
        layout.addStretch()

        self._phase = "IDLE"

    def update_phase(self, phase: str):
        phase = phase.strip().upper()
        color = PHASE_COLORS.get(phase, PHASE_COLORS["UNKNOWN"])
        self.phase_label.setText(phase)
        self.phase_label.setStyleSheet(
            f"background:{color}; color:#ffffff; border-radius:6px;"
        )
        self._phase = phase

    def update_health(self, bitmask: int):
        for name, bit in SENSOR_BITS.items():
            ok = bool(bitmask & (1 << bit))
            dot = self.sensor_dots[name]
            if ok:
                dot.setStyleSheet("background:#22aa44; color:#ffffff; border-radius:4px;")
            else:
                dot.setStyleSheet("background:#aa2222; color:#ffffff; border-radius:4px;")

    def update_value(self, key: str, value: float):
        if key not in self._value_rows:
            row = self._values_layout.rowCount()
            key_label = QLabel(key)
            key_label.setFont(QFont("Courier", 9))
            key_label.setStyleSheet("color:#aaaacc;")
            val_label = QLabel("—")
            val_label.setFont(QFont("Courier", 9, QFont.Bold))
            val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_label.setStyleSheet("color:#ffffff;")
            self._values_layout.addWidget(key_label, row, 0)
            self._values_layout.addWidget(val_label, row, 1)
            self._value_rows[key] = val_label
        self._value_rows[key].setText(f"{value:.4g}")

    def update_state(self, key: str, value: str):
        """Handle arbitrary !key:value state lines."""
        if key == "phase":
            self.update_phase(value)
        elif key == "health":
            try:
                self.update_health(int(value))
            except ValueError:
                pass
        elif key == "gps_fix":
            try:
                fix = int(value)
                label = GPS_FIX_LABELS.get(fix, str(fix))
                self.gps_fix_label.setText(label)
                if fix >= 3:
                    style = "background:#22aa44; color:#fff;"    # green — 3D fix
                elif fix >= 0:
                    style = "background:#aa8800; color:#fff;"    # amber — online, searching/2D
                else:
                    style = "background:#aa2222; color:#fff;"    # red — offline / init failed
                self.gps_fix_label.setStyleSheet(
                    style + " border-radius:3px; padding:0 4px;")
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
                    self.radio_label.setStyleSheet(
                        "background:#22aa44; color:#fff; border-radius:3px; padding:0 4px;")
                else:
                    self.radio_label.setText("OFFLINE")
                    self.radio_label.setStyleSheet(
                        "background:#aa2222; color:#fff; border-radius:3px; padding:0 4px;")
            except ValueError:
                pass

# ─── Main window ─────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apex Monitor")
        self.resize(1280, 820)
        self._apply_dark_theme()

        self._worker = SerialWorker()
        self._radio_worker = RadioWorker()
        self._t_start = None
        self._window_s = DEFAULT_WINDOW_S

        # Key → PlotGroupWidget mapping for routing
        self._key_to_group: dict[str, PlotGroupWidget] = {}
        self._overflow_group: PlotGroupWidget | None = None

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

        # Toolbar — fixed-height control bar with bottom separator
        root.addWidget(self._build_toolbar())

        # Main splitter: plots | state+log
        splitter = QSplitter(Qt.Horizontal)
        splitter.setContentsMargins(6, 6, 6, 6)
        root.addWidget(splitter)

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

        # Source selector
        self.source_combo = QComboBox()
        self.source_combo.addItem("USB Serial", "serial")
        self.source_combo.addItem("RTL-SDR Radio", "radio")
        self.source_combo.currentIndexChanged.connect(self._on_source_changed)
        layout.addWidget(QLabel("Source:"))
        layout.addWidget(self.source_combo)

        layout.addSpacing(8)

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
        self.radio_gain_input.setToolTip('RTL gain in dB, or "auto"')
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

        layout.addSpacing(12)

        # Connect / disconnect
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setFixedWidth(90)
        self.connect_btn.clicked.connect(self._toggle_connection)
        layout.addWidget(self.connect_btn)

        layout.addSpacing(12)

        # Time window
        layout.addWidget(QLabel("Window:"))
        self.window_spin = QSpinBox()
        self.window_spin.setRange(5, 300)
        self.window_spin.setValue(DEFAULT_WINDOW_S)
        self.window_spin.setSuffix(" s")
        self.window_spin.setFixedWidth(72)
        self.window_spin.valueChanged.connect(lambda v: setattr(self, "_window_s", v))
        layout.addWidget(self.window_spin)

        layout.addSpacing(12)

        # Clear button
        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(60)
        clear_btn.clicked.connect(self._clear_data)
        layout.addWidget(clear_btn)

        layout.addStretch()

        # Status indicator
        self.status_label = QLabel("Not connected")
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

        for title, keys, colors in PLOT_GROUPS:
            group = PlotGroupWidget(title, keys, colors)
            self._plot_groups.append(group)
            self._plot_layout.addWidget(group)
            for key in keys:
                self._key_to_group[key] = group

        self._plot_layout.addStretch()
        scroll.setWidget(container)
        return scroll

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
        self.log_view.setFont(QFont("Courier", 8))
        self.log_view.setStyleSheet("background:#0a0a14; color:#cccccc; border:none;")
        self.log_view.document().setMaximumBlockCount(2000)
        log_layout.addWidget(self.log_view)

        # Command input row
        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(4)
        self.cmd_input = CommandInput(KNOWN_COMMANDS)
        self.cmd_input.setPlaceholderText("Command (Tab completes, Up/Down history)")
        self.cmd_input.setFont(QFont("Courier", 8))
        self.cmd_input.setStyleSheet(
            "background:#0a0a14; color:#ccffcc; border:1px solid #334433;"
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
        self._radio_worker.status_changed.connect(self._on_status)
        self._radio_worker.finished.connect(self._on_worker_finished)
        self.state_panel.copy_requested.connect(self._copy_to_clipboard)

    # ── Serial control ────────────────────────────────────────────────────────

    def _source_mode(self) -> str:
        return self.source_combo.currentData() or "serial"

    def _on_source_changed(self):
        radio = self._source_mode() == "radio"
        serial_widgets = [
            self.port_label, self.port_combo, self.refresh_btn,
            self.baud_label, self.baud_combo,
        ]
        radio_widgets = [
            self.radio_freq_label, self.radio_freq_spin,
            self.radio_gain_label, self.radio_gain_input,
            self.radio_ppm_label, self.radio_ppm_spin,
            self.radio_rate_label, self.radio_rate_spin,
            self.radio_device_label, self.radio_device_input,
        ]
        for widget in serial_widgets:
            widget.setVisible(not radio)
        for widget in radio_widgets:
            widget.setVisible(radio)

        if hasattr(self, "cmd_input"):
            self.cmd_input.setEnabled(not radio)
            if radio:
                self.cmd_input.setPlaceholderText("Radio RX is receive-only; send RADIO_DATA_TEST over USB")
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
        else:
            if self._source_mode() == "radio":
                freq_hz = int(round(self.radio_freq_spin.value() * 1e6))
                sample_rate = int(self.radio_rate_spin.value())
                gain = self.radio_gain_input.text().strip() or "auto"
                ppm = int(self.radio_ppm_spin.value())
                device = self.radio_device_input.text().strip() or "0"
                self._radio_worker.configure(freq_hz, sample_rate, gain, ppm, device)
                self._radio_worker.start()
            else:
                idx  = self.port_combo.currentIndex()
                port = self.port_combo.itemData(idx) or self.port_combo.currentText().split()[0]
                baud = int(self.baud_combo.currentText())
                self._worker.configure(port, baud)
                self._worker.start()
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background:#aa2222; color:#ffffff;")
            self.source_combo.setEnabled(False)

    def _set_disconnected(self):
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet("")
        self.source_combo.setEnabled(True)

    def _on_worker_finished(self):
        if not self._worker.isRunning() and not self._radio_worker.isRunning():
            self._set_disconnected()

    def _send_command(self):
        text = self.cmd_input.text().strip()
        if not text:
            return
        if self._source_mode() == "radio":
            self._log("[monitor] Radio RX is receive-only; use USB serial to send commands")
            self.cmd_input.clear()
            return
        self._worker.send_bytes(text.encode() + b"\n")
        self._log(f"[TX] {text}")
        self.cmd_input.remember(text)
        self.cmd_input.clear()

    # ── Data routing ──────────────────────────────────────────────────────────

    def _on_line(self, line: str):
        now = time.monotonic()
        if self._t_start is None:
            self._t_start = now
        t = now - self._t_start

        if line.startswith(">"):
            # Numeric value
            try:
                key, val_str = line[1:].split(":", 1)
                value = float(val_str)
                group = self._key_to_group.get(key)
                if group is None:
                    group = self._get_or_create_overflow()
                    self._key_to_group[key] = group
                    group.add_key(key)
                group.push(key, t, value)
                self.state_panel.update_value(key, value)
            except (ValueError, IndexError):
                self._log(line)

        elif line.startswith("!"):
            # State / flag
            try:
                key, value = line[1:].split(":", 1)
                self.state_panel.update_state(key.strip(), value.strip())
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

        # Firmware plot-mode prefix: #ERROR: / #WARN: / #INFO:
        for level, (c, b) in self._LOG_STYLES.items():
            if text.startswith(level + ":"):
                color, bold = c, b
                text = text[len(level) + 1:].lstrip()
                prefix = f'<span style="color:{c};font-weight:{"bold" if b else "normal"};">[{level}]</span> '
                ts = f'<span style="color:#444466;">{time.strftime("%H:%M:%S")}</span> '
                self.log_view.append(ts + prefix + f'<span style="color:{color};">{text}</span>')
                sb = self.log_view.verticalScrollBar()
                sb.setValue(sb.maximum())
                return

        # [monitor] and [TX] internal messages
        if text.startswith("[monitor]"):
            color = "#446688"
        elif text.startswith("[TX]"):
            color = "#44ffaa"

        html = f'<span style="color:{color};font-weight:{"bold" if bold else "normal"};">{text}</span>'
        self.log_view.append(html)
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Plot refresh ──────────────────────────────────────────────────────────

    def _refresh_plots(self):
        for group in self._plot_groups:
            group.refresh(self._window_s)

    # ── Status ────────────────────────────────────────────────────────────────

    def _on_reconnect(self):
        """Teensy reset and re-enumerated — clear display so boot messages are visible."""
        self._clear_data()
        self._log("[monitor] Device reconnected — display cleared")

    def _on_status(self, msg: str, is_error: bool):
        color = "#dd4444" if is_error else "#44dd88"
        self.status_label.setText(msg)
        self.status_label.setStyleSheet(f"color:{color};")
        self._log(f"[monitor] {msg}")

    # ── Clipboard export ──────────────────────────────────────────────────────

    def _copy_to_clipboard(self):
        window_s = self.state_panel.copy_window_spin.value()
        lines = [
            f"# Apex Monitor — last {window_s}s",
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
        self._log(f"[monitor] Copied {n} data rows ({window_s}s window) to clipboard")

    # ── Util ──────────────────────────────────────────────────────────────────

    def _clear_data(self):
        self._t_start = None
        for group in self._plot_groups:
            for key in group._t:
                group._t[key].clear()
                group._y[key].clear()
            group._t0 = None
        self.log_view.clear()   # QTextEdit.clear()

    def _apply_dark_theme(self):
        self.setStyleSheet("""
            /* ── Base ─────────────────────────────────────────────────────── */
            QMainWindow, QWidget    { background: #0d0d1a; color: #ccccdd; }
            QLabel                  { color: #ccccdd; background: transparent; }
            QScrollArea             { border: none; }

            /* ── Toolbar ──────────────────────────────────────────────────── */
            QFrame#toolbar          { background: #111122;
                                      border-bottom: 1px solid #2a2a44; }
            QFrame#toolbar QLabel   { color: #9090b8; background: transparent; }

            /* Toolbar controls: inset look — darker than toolbar surface */
            QFrame#toolbar QComboBox,
            QFrame#toolbar QSpinBox,
            QFrame#toolbar QDoubleSpinBox,
            QFrame#toolbar QLineEdit { background: #09091a; border: 1px solid #2e2e50;
                                       color: #ccccdd; padding: 2px 6px; border-radius: 3px; }
            QFrame#toolbar QPushButton
                                    { background: #09091a; border: 1px solid #2e2e50;
                                      color: #aaaacc; padding: 3px 10px; border-radius: 3px; }
            QFrame#toolbar QPushButton:hover
                                    { background: #14142a; border-color: #4444aa;
                                      color: #ddddff; }

            /* ── Panel controls (outside toolbar) ────────────────────────── */
            QGroupBox               { border: 1px solid #333355; border-radius: 4px;
                                      margin-top: 6px; padding-top: 6px; color: #8888aa; }
            QGroupBox::title        { subcontrol-origin: margin; left: 8px; }

            QComboBox, QSpinBox,
            QDoubleSpinBox, QLineEdit
                                    { background: #1a1a2e; border: 1px solid #444466;
                                      color: #ccccdd; padding: 2px 6px; border-radius: 3px; }
            QPushButton             { background: #1e1e3a; border: 1px solid #444466;
                                      color: #ccccdd; padding: 3px 10px; border-radius: 3px; }
            QPushButton:hover       { background: #2a2a4a; }

            /* ── ComboBox arrow + dropdown popup ─────────────────────────── */
            QComboBox::drop-down    { subcontrol-origin: padding;
                                      subcontrol-position: top right;
                                      width: 18px;
                                      border-left: 1px solid #303050;
                                      border-radius: 0 3px 3px 0; }
            QComboBox::down-arrow   { border-left:  4px solid transparent;
                                      border-right: 4px solid transparent;
                                      border-top:   5px solid #8888cc;
                                      width: 0; height: 0; }
            QComboBox::down-arrow:disabled
                                    { border-top-color: #444455; }

            QComboBox QAbstractItemView {
                                      background: #1a1a2e;
                                      border: 1px solid #444466;
                                      color: #ccccdd;
                                      selection-background-color: #2a2a50;
                                      selection-color: #ffffff;
                                      outline: none; }

            /* ── SpinBox arrows ───────────────────────────────────────────── */
            QSpinBox::up-button     { subcontrol-origin: border;
                                      subcontrol-position: top right;
                                      width: 16px;
                                      border-left: 1px solid #303050;
                                      border-bottom: 1px solid #303050;
                                      background: transparent; }
            QSpinBox::down-button   { subcontrol-origin: border;
                                      subcontrol-position: bottom right;
                                      width: 16px;
                                      border-left: 1px solid #303050;
                                      background: transparent; }
            QSpinBox::up-arrow      { border-left:   3px solid transparent;
                                      border-right:  3px solid transparent;
                                      border-bottom: 4px solid #8888cc;
                                      width: 0; height: 0; }
            QSpinBox::down-arrow    { border-left:  3px solid transparent;
                                      border-right: 3px solid transparent;
                                      border-top:   4px solid #8888cc;
                                      width: 0; height: 0; }
            QSpinBox::up-arrow:disabled,
            QSpinBox::down-arrow:disabled
                                    { border-bottom-color: #444455;
                                      border-top-color:    #444455; }

            /* ── Scrollbars ───────────────────────────────────────────────── */
            QScrollBar:vertical     { background: #0d0d1a; width: 8px;
                                      margin: 0; border: none; }
            QScrollBar::handle:vertical
                                    { background: #2e2e50; border-radius: 4px;
                                      min-height: 24px; }
            QScrollBar::handle:vertical:hover
                                    { background: #44447a; }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical  { height: 0; }
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical  { background: none; }

            QScrollBar:horizontal   { background: #0d0d1a; height: 8px;
                                      margin: 0; border: none; }
            QScrollBar::handle:horizontal
                                    { background: #2e2e50; border-radius: 4px;
                                      min-width: 24px; }
            QScrollBar::handle:horizontal:hover
                                    { background: #44447a; }
            QScrollBar::add-line:horizontal,
            QScrollBar::sub-line:horizontal { width: 0; }
            QScrollBar::add-page:horizontal,
            QScrollBar::sub-page:horizontal { background: none; }
        """)
        pg.setConfigOption("background", "#0d0d1a")
        pg.setConfigOption("foreground", "#888899")

    def closeEvent(self, event):
        self._worker.stop()
        self._radio_worker.stop()
        event.accept()


# ─── Entry point ─────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Apex Monitor")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
