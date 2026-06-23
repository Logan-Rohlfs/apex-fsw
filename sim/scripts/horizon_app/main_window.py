"""HORIZON main window — assembly, page routing, connection/run control, theme.

The connection to the APEX FC / SDR is a persistent adaptive bar (connection.py)
above the page tabs; each page's toolbar holds only its view/run controls. HIL
no longer auto-starts on connect — it has an explicit Start button. The data
routing, plot refresh, and firmware-facing handlers are relocated verbatim from
the original horizon.py; only the chrome (connection, toolbar, theme) is new.
"""

from __future__ import annotations

import html
import sys
import time
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg

from PyQt5.QtCore import Qt, QTimer, QSettings
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSplitter, QScrollArea, QFrame,
    QTextEdit, QGroupBox, QSpinBox, QDoubleSpinBox, QCheckBox, QLineEdit,
    QTabBar, QFileDialog,
)

from . import theme
from .theme import mono_font, HorizonStyle
from .constants import (
    PAGES, PLOT_GROUPS, RADIO_PLOT_GROUPS, HIL_PLOT_GROUPS, HIL_EVENT_STYLES,
    KNOWN_COMMANDS, PLOT_UPDATE_HZ, DEFAULT_WINDOW_S,
)
from .widgets import PlotGroupWidget, SpectrumPanel, CommandInput
from .state_panel import StatePanel
from .connection import ConnectionBar
from .workers import SerialWorker, RadioWorker, HilWorker, LogOpsWorker, ReplayWorker
from .log_panel import LogPanelMixin
from .paths import _SIM_ROOT

_SETTINGS_ORG = "SpaceRaiders"
_SETTINGS_APP = "HORIZON"


class MainWindow(QMainWindow, LogPanelMixin):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HORIZON — Apex Ground")
        self.resize(1280, 820)

        # Restore the last-used theme (dark default) before building widgets so
        # everything constructs in the right palette.
        settings = QSettings(_SETTINGS_ORG, _SETTINGS_APP)
        theme.set_mode(str(settings.value("theme_mode", theme.DEFAULT_MODE)))

        self._worker = SerialWorker()
        self._radio_worker = RadioWorker()
        self._hil_worker = HilWorker()
        self._log_ops = LogOpsWorker()
        self._replay_worker = ReplayWorker()
        self._log_export_after_pull = False
        self._last_export_dir = None
        self._t_start = None
        self._window_s = DEFAULT_WINDOW_S

        # Key → PlotGroupWidget mapping for routing, per source
        self._key_to_group_serial: dict = {}
        self._key_to_group_radio: dict = {}
        self._key_to_group_hil: dict = {}
        self._overflow_group = None
        self._device_entries: list = []
        self._visible_device_entries: list = []
        self._local_log_info_cache: dict = {}

        # Debounced restart so gain/freq/ppm changes apply while connected
        self._retune_timer = QTimer(self)
        self._retune_timer.setSingleShot(True)
        self._retune_timer.setInterval(500)
        self._retune_timer.timeout.connect(self._apply_radio_retune)

        self._build_ui()
        self._connect_signals()
        self._apply_theme()

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

        # Persistent connection bar — the FC/SDR link, shared across pages, with
        # branding and the theme toggle. Decoupled from the page tabs.
        self.conn = ConnectionBar()
        root.addWidget(self.conn)

        # Alias the connection widgets onto self so the relocated handlers and
        # the radio/HIL start logic find them where they always lived.
        self.port_combo = self.conn.port_combo
        self.baud_combo = self.conn.baud_combo
        self.refresh_btn = self.conn.refresh_btn
        self.radio_freq_spin = self.conn.radio_freq_spin
        self.radio_gain_input = self.conn.radio_gain_input
        self.radio_ppm_spin = self.conn.radio_ppm_spin
        self.radio_rate_spin = self.conn.radio_rate_spin
        self.radio_device_input = self.conn.radio_device_input
        self.hil_fake_check = self.conn.hil_fake_check
        self.connect_btn = self.conn.connect_btn
        self.status_label = self.conn.status_label
        self._refresh_ports()

        # Tab strip (page switcher only — branding moved to the connection bar).
        tab_strip = QFrame()
        tab_strip.setObjectName("tabstrip")
        tab_strip.setFixedHeight(30)
        tl = QHBoxLayout(tab_strip)
        tl.setContentsMargins(10, 0, 10, 0)
        tl.setSpacing(0)
        self.tab_bar = QTabBar()
        self.tab_bar.setExpanding(False)
        self.tab_bar.setDrawBase(False)
        self.tab_bar.setFocusPolicy(Qt.NoFocus)
        for title, _mode in PAGES:
            self.tab_bar.addTab(title)
        self.tab_bar.currentChanged.connect(self._on_source_changed)
        tl.addWidget(self.tab_bar)
        tl.addStretch()
        root.addWidget(tab_strip)

        # Page toolbar — view/run controls only (no connection widgets).
        root.addWidget(self._build_toolbar())

        # Main splitter: plots | state + log
        splitter = QSplitter(Qt.Horizontal)
        splitter.setContentsMargins(6, 6, 6, 6)
        root.addWidget(splitter, stretch=1)
        splitter.addWidget(self._build_plot_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setSizes([860, 260])
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

        self._on_source_changed()

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("toolbar")
        bar.setFixedHeight(42)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(6)

        # ── HIL run controls — the run starts here (not on connect) ───────────
        self.hil_start_btn = QPushButton("▶ Start run")
        self.hil_start_btn.setFixedWidth(104)
        self.hil_start_btn.setToolTip("Start the HIL closed-loop run with the options below.")
        self.hil_start_btn.clicked.connect(self._on_hil_start)
        layout.addWidget(self.hil_start_btn)

        self.hil_stop_btn = QPushButton("■ Stop")
        self.hil_stop_btn.setFixedWidth(64)
        self.hil_stop_btn.clicked.connect(self._stop_active)
        layout.addWidget(self.hil_stop_btn)

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
        self.hil_pad_spin.setRange(0.0, 1200.0)
        self.hil_pad_spin.setDecimals(0)
        self.hil_pad_spin.setSingleStep(5.0)
        self.hil_pad_spin.setValue(6.0)
        self.hil_pad_spin.setSuffix(" s")
        self.hil_pad_spin.setFixedWidth(90)
        self.hil_pad_spin.setToolTip(
            "Seconds the FC sits on the pad with arm switches closed before\n"
            "liftoff. The FC arms itself a few seconds in (its own\n"
            "IDLE→ARMED gate) and then sits ARMED on real sensor data for\n"
            "the rest of this time. At 1x this is real time.")
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
            "full-flight HIL landing. Exercises LANDED handling and\n"
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

        self._hil_toolbar_widgets = [
            self.hil_start_btn, self.hil_stop_btn,
            self.hil_speed_label, self.hil_speed_spin, self.hil_noise_check,
            self.hil_seed_label, self.hil_seed_input,
            self.hil_delay_label, self.hil_delay_spin,
            self.hil_pad_label, self.hil_pad_spin,
            self.hil_post_label, self.hil_post_spin,
            self.hil_record_check, self.hil_full_check,
        ]

        layout.addStretch()

        # Logs page has no live connection — state the pipeline instead.
        self.logs_hint_label = QLabel(
            "Flight Computer  →  Laptop Archive  →  CSV Exports")
        self.logs_hint_label.setFont(mono_font(9))
        layout.addWidget(self.logs_hint_label)

        # Time window
        self.window_label = QLabel("Window:")
        layout.addWidget(self.window_label)
        self.window_spin = QSpinBox()
        self.window_spin.setRange(5, 300)
        self.window_spin.setValue(DEFAULT_WINDOW_S)
        self.window_spin.setSuffix(" s")
        self.window_spin.setFixedWidth(72)
        self.window_spin.valueChanged.connect(lambda v: setattr(self, "_window_s", v))
        layout.addWidget(self.window_label)
        layout.addWidget(self.window_spin)

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.setFixedWidth(72)
        self.clear_btn.clicked.connect(self._clear_data)
        layout.addWidget(self.clear_btn)

        # ── Log replay (Sensors page only) — stream a decoded flight CSV ──────
        self.replay_speed_combo = QComboBox()
        for label, mult in (("1×", 1.0), ("2×", 2.0), ("5×", 5.0),
                            ("20×", 20.0), ("Max", 0.0)):
            self.replay_speed_combo.addItem(label, mult)
        self.replay_speed_combo.setFixedWidth(64)
        self.replay_speed_combo.setToolTip("Replay speed (× real time; Max = no pacing)")
        self.replay_btn = QPushButton("▶ Replay log…")
        self.replay_btn.setFixedWidth(120)
        self.replay_btn.setToolTip(
            "Stream a decoded flight-log CSV through the plots as if it were live.")
        self.replay_btn.clicked.connect(self._toggle_replay)
        self._replay_widgets = [self.replay_speed_combo, self.replay_btn]
        layout.addWidget(self.replay_speed_combo)
        layout.addWidget(self.replay_btn)

        return bar

    # ── Page routing ────────────────────────────────────────────────────────
    def _source_mode(self) -> str:
        idx = self.tab_bar.currentIndex() if hasattr(self, "tab_bar") else 0
        return PAGES[idx][1] if 0 <= idx < len(PAGES) else "serial"

    def _on_source_changed(self):
        mode = self._source_mode()
        radio = mode == "radio"
        hil = mode == "hil"
        logs = mode == "logs"
        hil_fake = hil and self.hil_fake_check.isChecked()

        # Connection bar adapts to the active page's device.
        self.conn.set_mode(mode, self.hil_fake_check.isChecked())

        # Toolbar: HIL run controls only on HIL; window/clear off on Logs.
        for w in self._hil_toolbar_widgets:
            w.setVisible(hil)
        self.hil_compare_check.setVisible(hil and not hil_fake)
        for w in (self.window_label, self.window_spin, self.clear_btn):
            w.setVisible(not logs)
        # Replay drives the serial/Sensors plots, so offer it only there.
        for w in self._replay_widgets:
            w.setVisible(mode == "serial")
        self.logs_hint_label.setVisible(logs)
        if hil:
            self._update_hil_buttons()

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

    # ── Connection / run control ──────────────────────────────────────────────
    def _toggle_connection(self):
        # The connection button drives serial/radio/logs only; HIL runs from the
        # toolbar Start button.
        if self._worker.isRunning():
            self._worker.stop()
            self._set_disconnected()
            return
        if self._radio_worker.isRunning():
            self._radio_worker.stop()
            self._set_disconnected()
            return
        if self._hil_worker.isRunning():
            self._hil_worker.stop()
            self._set_disconnected()
            return

        mode = self._source_mode()
        if mode == "radio":
            self._start_radio()
        else:
            # serial OR logs — both talk to the FC over USB serial.
            idx = self.port_combo.currentIndex()
            port = self.port_combo.itemData(idx) or self.port_combo.currentText().split()[0]
            baud = int(self.baud_combo.currentText())
            self._worker.configure(port, baud)
            self._worker.start()
        self.conn.set_status(True, "Connecting…")

    def _on_hil_start(self):
        if self._hil_worker.isRunning():
            return
        self._start_hil()
        self._update_hil_buttons()

    def _stop_active(self):
        for w in (self._worker, self._radio_worker, self._hil_worker,
                  self._replay_worker):
            if w.isRunning():
                w.stop()
        self._set_disconnected()

    def _update_hil_buttons(self):
        running = self._hil_worker.isRunning()
        self.hil_start_btn.setEnabled(not running)
        self.hil_stop_btn.setEnabled(running)

    def _set_disconnected(self):
        self.conn.set_status(False, "Not connected")
        self._update_hil_buttons()

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
        self._replay_worker.line_received.connect(self._on_line)
        self._replay_worker.status_changed.connect(self._on_status)
        self._replay_worker.progress.connect(self._on_replay_progress)
        self._replay_worker.finished.connect(self._on_replay_finished)
        self._log_ops.progress.connect(self._append_log_decode_text)
        self._log_ops.done.connect(self._on_log_job_done)
        self._log_ops.failed.connect(self._on_log_job_failed)
        self.state_panel.copy_requested.connect(self._copy_to_clipboard)

        self.conn.connect_clicked.connect(self._toggle_connection)
        self.conn.refresh_clicked.connect(self._refresh_ports)
        self.conn.theme_toggle_clicked.connect(self._toggle_theme)
        self.conn.radio_param_changed.connect(self._schedule_radio_retune)
        self.conn.hil_fake_toggled.connect(self._on_source_changed)

    # ── Status ────────────────────────────────────────────────────────────────
    def _on_status(self, msg: str, is_error: bool):
        self.status_label.setText(msg)
        self.status_label.setStyleSheet(f"color:{theme.BAD if is_error else theme.GOOD};")
        self._log(f"[horizon] {msg}")

    # ── Theme ─────────────────────────────────────────────────────────────────
    def _toggle_theme(self):
        new_mode = "light" if theme.is_dark() else "dark"
        theme.set_mode(new_mode)
        QSettings(_SETTINGS_ORG, _SETTINGS_APP).setValue("theme_mode", new_mode)
        self._apply_theme()

    def _apply_theme(self):
        self.setStyleSheet(theme.build_qss())
        pg.setConfigOption("background", theme.BG)
        pg.setConfigOption("foreground", theme.TEXT_DIM)

        self.conn.apply_theme()
        if hasattr(self, "state_panel"):
            self.state_panel.apply_theme()
        if hasattr(self, "spectrum_panel"):
            self.spectrum_panel.apply_theme()
        for group in getattr(self, "_plot_groups", []):
            group.apply_theme()
        if hasattr(self, "log_view"):
            self.log_view.setStyleSheet(
                f"background:{theme.INSET}; color:{theme.TEXT}; border:none;")
        if hasattr(self, "cmd_input"):
            cmd_col = "#ccffcc" if theme.is_dark() else "#2f7d3a"
            self.cmd_input.setStyleSheet(
                f"background:{theme.INSET}; color:{cmd_col};"
                f" border:1px solid {theme.BORDER}; border-radius:3px; padding:2px 4px;")
        if hasattr(self, "logs_hint_label"):
            self.logs_hint_label.setStyleSheet(f"color:{theme.TEXT_DIM};")

    def closeEvent(self, event):
        self._worker.stop()
        self._radio_worker.stop()
        self._hil_worker.stop()
        self._log_ops.wait(1000)   # jobs are short except MTP; don't hang exit
        event.accept()

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
        self.log_view.setStyleSheet(f"background:{theme.INSET}; color:#cccccc; border:none;")
        self.log_view.document().setMaximumBlockCount(2000)
        log_layout.addWidget(self.log_view)

        # Command input row
        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(4)
        self.cmd_input = CommandInput(KNOWN_COMMANDS)
        self.cmd_input.setPlaceholderText("Command (Tab completes, Up/Down history)")
        self.cmd_input.setFont(mono_font(8))
        self.cmd_input.setStyleSheet(
            f"background:{theme.INSET}; color:#ccffcc; border:1px solid {theme.BORDER};"
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

    def _on_worker_finished(self):
        if (not self._worker.isRunning() and not self._radio_worker.isRunning()
                and not self._hil_worker.isRunning()):
            self._set_disconnected()

    # ── Log replay ────────────────────────────────────────────────────────────
    def _toggle_replay(self):
        if self._replay_worker.isRunning():
            self._replay_worker.stop()
            return

        default_dir = _SIM_ROOT / "output" / "log_exports"
        start_dir = str(default_dir if default_dir.exists() else _SIM_ROOT)
        path, _ = QFileDialog.getOpenFileName(
            self, "Replay flight log", start_dir, "Decoded log (*.csv);;All files (*)")
        if not path:
            return

        # Replay drives the serial/Sensors plots — make sure that page is active.
        self.tab_bar.setCurrentIndex(0)
        self._clear_data()
        speed = self.replay_speed_combo.currentData()
        self._replay_worker.configure(path, speed)
        self._replay_worker.start()
        self.replay_btn.setText("■ Stop replay")
        self.conn.set_status(True, f"Replaying {Path(path).name}")

    def _on_replay_progress(self, frac: float):
        self.conn.set_status(True, f"Replaying… {frac * 100:.0f}%")

    def _on_replay_finished(self):
        self.replay_btn.setText("▶ Replay log…")
        if not self._any_source_running():
            self._set_disconnected()

    def _any_source_running(self) -> bool:
        return (self._worker.isRunning() or self._radio_worker.isRunning()
                or self._hil_worker.isRunning() or self._replay_worker.isRunning())

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

# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app._horizon_style = HorizonStyle(app.style())
    app.setStyle(app._horizon_style)
    app.setApplicationName("HORIZON — Apex Ground")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
