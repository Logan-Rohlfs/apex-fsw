#!/usr/bin/env python3
"""
Apex Flight Computer — Ground Monitor

Two data sources, switched in the toolbar:
  USB Serial   — laptop-connected Teensy (APEX_MONITOR build), full sensor set
  RTL-SDR      — 2-GFSK telemetry downlink (441.480 MHz) with live spectrum,
                 waterfall, ground-station plot layout, and link stats

Both sources speak the same line protocol:
  >key:value    numeric — routed to live plots and values table
  !key:value    state   — routed to state panel (phase, health, link, etc.)
  #LEVEL: msg   log     — shown in the log panel with color coding

Commands (USB serial only): ARM, DISARM, TELEM_ON, TELEM_OFF,
RADIO_DATA_TEST, RADIO_MARKER.

Run: python scripts/monitor.py
"""

import html
import sys
import time
import shutil
import subprocess
import threading
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSplitter, QScrollArea,
    QFrame, QTextEdit, QGridLayout, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QLineEdit, QCompleter,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF
from PyQt5.QtGui import QFont

from radio_gfsk_rx import (
    BITRATE_BPS,
    DEFAULT_FREQ_HZ,
    DEFAULT_SAMPLE_RATE_HZ,
    DEVIATION_HZ,
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
    ("Fusion",    ["alt_agl", "velocity", "pred_apogee"],  ["#00d4ff", "#ff6b35", "#a8ff3e"]),
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

KNOWN_COMMANDS = [
    "ARM",
    "DISARM",
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
        self.line_received.emit("#INFO: radio rx waiting for RADIO_DATA_TEST frames (2-GFSK)")

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
        try:
            samples = load_u8_iq(raw)
            frames = find_frames(samples, self._sample_rate)
        except Exception as exc:
            self.line_received.emit(f"#WARN: radio rx decode error: {exc}")
            return

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

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.plot = PlotWidget(background=SURFACE)
        self.plot.setMinimumHeight(160)
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
        curve = self.plot.plot(pen=pg.mkPen(color, width=1.5), name=key)
        curve.setDownsampling(auto=True, method="peak")
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

    def refresh(self, window_s: int, t_now: float = None):
        if window_s != self._last_window:
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

    def clear(self):
        for key in self.keys:
            self._t[key].clear()
            self._y[key].clear()
            self.curves[key].setData([], [])
        self._t0 = None
        self._last_push = None
        self._dirty.clear()

    def add_key(self, key: str, color: str = "#ffffff"):
        """Dynamically add a key not in the original definition."""
        if key in self._t:
            return
        self.keys.append(key)
        self.colors.append(color)
        self._t[key] = deque(maxlen=MAX_POINTS)
        self._y[key] = deque(maxlen=MAX_POINTS)
        self._make_curve(key, color)

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

        layout.addWidget(link_box)
        self.set_link_mode(radio=False)

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

    def set_link_mode(self, radio: bool):
        """Show TX stats (USB serial) or RX packet stats (SDR) in the Link box."""
        for w in self._link_serial_widgets:
            w.setVisible(not radio)
        for w in self._link_radio_widgets:
            w.setVisible(radio)

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

    def flush_values(self):
        if not self._pending_values:
            return
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
        self.setWindowTitle("Apex Monitor")
        self.resize(1280, 820)
        self._apply_dark_theme()

        self._worker = SerialWorker()
        self._radio_worker = RadioWorker()
        self._t_start = None
        self._window_s = DEFAULT_WINDOW_S

        # Key → PlotGroupWidget mapping for routing, per source
        self._key_to_group_serial: dict[str, PlotGroupWidget] = {}
        self._key_to_group_radio: dict[str, PlotGroupWidget] = {}
        self._overflow_group: PlotGroupWidget | None = None

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

        if hasattr(self, "spectrum_panel"):
            self.spectrum_panel.setVisible(radio)
            for group in self._serial_groups:
                group.setVisible(not radio)
            for group in self._radio_groups:
                group.setVisible(radio)

        if hasattr(self, "state_panel"):
            self.state_panel.set_link_mode(radio)

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
                self._start_radio()
            else:
                idx  = self.port_combo.currentIndex()
                port = self.port_combo.itemData(idx) or self.port_combo.currentText().split()[0]
                baud = int(self.baud_combo.currentText())
                self._worker.configure(port, baud)
                self._worker.start()
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet(
                f"background:{BAD}; color:#ffffff; border:1px solid #cc4444;")
            self.source_combo.setEnabled(False)

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
        self._log("[monitor] Radio settings changed — retuning…")
        self._radio_worker.stop()
        self._start_radio()

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
                key_map = (self._key_to_group_radio if self._source_mode() == "radio"
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
            # [monitor] and [TX] internal messages
            if text.startswith("[monitor]"):
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
            group.refresh(self._window_s, t_now)
        self.state_panel.flush_values()

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
            group.clear()
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
            QDoubleSpinBox, QLineEdit
                                    {{ background: {INSET}; border: 1px solid {BORDER_HI};
                                       color: {TEXT}; padding: 2px 6px; border-radius: 3px; }}
            QComboBox:focus, QSpinBox:focus,
            QDoubleSpinBox:focus, QLineEdit:focus
                                    {{ border-color: {ACCENT}; }}
            QPushButton             {{ background: #1e1e3a; border: 1px solid {BORDER_HI};
                                       color: {TEXT}; padding: 3px 10px; border-radius: 3px; }}
            QPushButton:hover       {{ background: #2a2a4a; border-color: {ACCENT}; }}
            QPushButton:pressed     {{ background: #16162e; }}

            /* ── ComboBox dropdown + popup ─────────────────────────────────
               Leave arrow glyphs to Qt's native style. Qt stylesheets do not
               reliably support CSS border triangles on all platforms, which
               can render as empty symbol boxes instead of arrows. */
            QComboBox::drop-down    {{ subcontrol-origin: padding;
                                       subcontrol-position: top right;
                                       width: 18px;
                                       border-left: 1px solid #303050;
                                       border-radius: 0 3px 3px 0; }}

            QComboBox QAbstractItemView {{
                                       background: {SURFACE};
                                       border: 1px solid {BORDER_HI};
                                       color: {TEXT};
                                       selection-background-color: #2a2a50;
                                       selection-color: #ffffff;
                                       outline: none; }}

            /* ── SpinBox arrow button frames ────────────────────────────────
               As with combo boxes, use native Qt arrow drawing for the glyphs
               so no icon font or platform-specific symbol is required. */
            QSpinBox::up-button     {{ subcontrol-origin: border;
                                       subcontrol-position: top right;
                                       width: 16px;
                                       border-left: 1px solid #303050;
                                       border-bottom: 1px solid #303050;
                                       background: transparent; }}
            QSpinBox::down-button   {{ subcontrol-origin: border;
                                       subcontrol-position: bottom right;
                                       width: 16px;
                                       border-left: 1px solid #303050;
                                       background: transparent; }}
            QDoubleSpinBox::up-button
                                    {{ subcontrol-origin: border;
                                       subcontrol-position: top right;
                                       width: 16px;
                                       border-left: 1px solid #303050;
                                       border-bottom: 1px solid #303050;
                                       background: transparent; }}
            QDoubleSpinBox::down-button
                                    {{ subcontrol-origin: border;
                                       subcontrol-position: bottom right;
                                       width: 16px;
                                       border-left: 1px solid #303050;
                                       background: transparent; }}

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
