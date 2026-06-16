"""Right-hand state panel: phase, flight numbers, airbrake gauge, health,
interlocks, radio/link/GPS status, and the live values table.

Ported from horizon.py. Structural label colors route through `theme` and are
re-applied by apply_theme(); live status badges (phase, GPS fix, health dots)
re-color on the next update line after a mode switch.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSpinBox,
)

from . import theme
from .theme import mono_font, badge_style
from .widgets import DeploymentGauge
from .constants import (
    PHASE_COLORS, SENSOR_BITS, GPS_FIX_LABELS, FT_PER_M,
    TARGET_APOGEE_M, MACH_GATE_MPS, DEFAULT_WINDOW_S,
)


def _off_badge() -> str:
    return badge_style(theme.BORDER, theme.TEXT_DIM)


class StatePanel(QWidget):
    copy_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setFixedWidth(260)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        self._dim_labels = []   # key/title labels (TEXT_DIM)
        self._val_labels = []   # value labels (TEXT)

        # Phase badge
        self.phase_label = QLabel("IDLE")
        self.phase_label.setAlignment(Qt.AlignCenter)
        self.phase_label.setFont(mono_font(22, bold=True))
        self.phase_label.setFixedHeight(54)
        self.phase_label.setStyleSheet(badge_style(PHASE_COLORS["IDLE"], radius=6))
        layout.addWidget(self.phase_label)

        # Flight info — converted/derived values in flight units.
        flight_box = QGroupBox("Flight")
        flight_grid = QGridLayout(flight_box)
        flight_grid.setContentsMargins(6, 4, 6, 4)
        flight_grid.setVerticalSpacing(2)
        self._flight_labels: dict = {}

        def flight_row(row: int, title: str, key: str, size: int = 10):
            k = QLabel(title)
            k.setFont(mono_font(9))
            k.setStyleSheet(f"color:{theme.TEXT_DIM};")
            v = QLabel("—")
            v.setFont(mono_font(size, bold=True))
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            v.setStyleSheet(f"color:{theme.TEXT};")
            flight_grid.addWidget(k, row, 0)
            flight_grid.addWidget(v, row, 1)
            self._flight_labels[key] = v
            self._dim_labels.append(k)
            self._val_labels.append(v)

        flight_row(0, "Alt",         "f_alt", size=12)
        flight_row(1, "Max alt",     "f_max")
        flight_row(2, "Pred apogee", "f_pred", size=12)
        flight_row(3, "vs 10k ft",   "f_err")
        flight_row(4, "Speed",       "f_speed", size=12)
        flight_row(5, "Deploy gate", "f_gate")
        self._flight_labels["f_gate"].setText(f"< {MACH_GATE_MPS:.0f} m/s")
        self._flight_labels["f_gate"].setStyleSheet(f"color:{theme.TEXT_DIM};")
        self._val_labels.remove(self._flight_labels["f_gate"])
        self._dim_labels.append(self._flight_labels["f_gate"])
        layout.addWidget(flight_box)

        self._flight_alt_m = None      # latest values, flushed on UI tick
        self._flight_max_m = 0.0
        self._flight_pred_m = None
        self._flight_speed = None

        # Airbrake deployment gauge
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
        self.sensor_dots: dict = {}
        for name in SENSOR_BITS:
            dot = QLabel(name)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(mono_font(9, bold=True))
            dot.setFixedSize(52, 22)
            dot.setStyleSheet(_off_badge())
            self.sensor_dots[name] = dot
            health_layout.addWidget(dot)
        layout.addWidget(health_box)

        # System health occupies the upper nibble of the radio health byte.
        self.system_health_box = QGroupBox("Systems")
        system_health_layout = QHBoxLayout(self.system_health_box)
        system_health_layout.setContentsMargins(6, 4, 6, 4)
        system_health_layout.setSpacing(3)
        self.system_health_dots: dict = {}
        for name in ("GPS", "RAD", "QSPI", "SD"):
            dot = QLabel(name)
            dot.setAlignment(Qt.AlignCenter)
            dot.setFont(mono_font(8, bold=True))
            dot.setFixedSize(52, 22)
            dot.setStyleSheet(_off_badge())
            self.system_health_dots[name] = dot
            system_health_layout.addWidget(dot)
        layout.addWidget(self.system_health_box)

        # Operational flags share the upper five bits of phase_status.
        self.radio_ops_box = QGroupBox("Flight Interlocks")
        radio_ops_grid = QGridLayout(self.radio_ops_box)
        radio_ops_grid.setContentsMargins(6, 4, 6, 4)
        radio_ops_grid.setHorizontalSpacing(3)
        radio_ops_grid.setVerticalSpacing(3)
        self.radio_ops_dots: dict = {}
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
            dot.setStyleSheet(_off_badge())
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
        self.radio_label.setStyleSheet(badge_style(theme.BAD, radius=3))
        radio_layout.addWidget(self.radio_label)
        layout.addWidget(radio_box)

        # Link stats — TX side over USB serial, RX side over the SDR
        link_box = QGroupBox("Link")
        link_grid = QGridLayout(link_box)
        link_grid.setContentsMargins(6, 4, 6, 4)
        link_grid.setVerticalSpacing(2)
        self._link_value_labels: dict = {}
        self._link_serial_widgets: list = []
        self._link_radio_widgets: list = []
        self._link_hil_widgets: list = []

        def link_row(row: int, title: str, key: str, widgets: list):
            k = QLabel(title)
            k.setFont(mono_font(9))
            k.setStyleSheet(f"color:{theme.TEXT_DIM};")
            v = QLabel("—")
            v.setFont(mono_font(9, bold=True))
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            v.setStyleSheet(f"color:{theme.TEXT};")
            link_grid.addWidget(k, row, 0)
            link_grid.addWidget(v, row, 1)
            widgets.extend((k, v))
            self._link_value_labels[key] = v
            self._dim_labels.append(k)
            self._val_labels.append(v)

        # Serial mode: what the flight computer reports about its own TX
        link_row(0, "Beacon",  "telem",      self._link_serial_widgets)
        link_row(1, "TX seq",  "tx_seq",     self._link_serial_widgets)
        link_row(2, "Sent",    "tx_sent",    self._link_serial_widgets)
        link_row(3, "Skipped", "tx_skipped", self._link_serial_widgets)
        # Radio mode: what the SDR receiver measures
        link_row(4, "RX seq",  "rx_seq",     self._link_radio_widgets)
        link_row(5, "Rate",    "rx_rate",     self._link_radio_widgets)
        link_row(6, "Loss",    "rx_loss",     self._link_radio_widgets)
        link_row(7, "Quality", "rx_quality",  self._link_radio_widgets)
        link_row(8, "Offset",  "rx_offset",   self._link_radio_widgets)
        link_row(9, "Packets", "rx_count",    self._link_radio_widgets)
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
        self.gps_fix_label.setStyleSheet(_off_badge() + " border-radius:3px;")

        self.gps_sats_label = QLabel("0 sats")
        self.gps_sats_label.setFont(mono_font(9))
        self.gps_sats_label.setStyleSheet(f"color:{theme.TEXT_DIM};")

        self.gps_utc_label = QLabel("—")
        self.gps_utc_label.setFont(mono_font(8))
        self.gps_utc_label.setStyleSheet(f"color:{theme.TEXT_DIM};")

        gps_layout.addWidget(self.gps_fix_label)
        gps_layout.addWidget(self.gps_sats_label)
        gps_layout.addWidget(self.gps_utc_label, stretch=1)
        self._dim_labels.extend((self.gps_sats_label, self.gps_utc_label))
        layout.addWidget(gps_box)

        # Values table with copy button in header
        values_box = QGroupBox("Values")
        values_outer = QVBoxLayout(values_box)
        values_outer.setContentsMargins(6, 4, 6, 4)
        values_outer.setSpacing(4)

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
        self._value_rows: dict = {}
        self._pending_values: dict = {}
        values_outer.addWidget(self._values_grid)
        layout.addWidget(values_box)
        layout.addStretch()

    def apply_theme(self):
        """Re-skin structural labels after a light/dark switch. Live status
        badges re-color on the next update line."""
        for lbl in self._dim_labels:
            lbl.setStyleSheet(f"color:{theme.TEXT_DIM};")
        for lbl in self._val_labels:
            lbl.setStyleSheet(f"color:{theme.TEXT};")
        self.update_phase(self.phase_label.text())
        self.deploy_gauge.apply_theme()
        for dot in list(self.system_health_dots.values()) + list(self.radio_ops_dots.values()):
            dot.setStyleSheet(_off_badge())

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
        dot.setStyleSheet(badge_style(theme.GOOD if ok else theme.BAD))

    def update_phase(self, phase: str):
        phase = phase.strip().upper()
        color = PHASE_COLORS.get(phase, PHASE_COLORS["UNKNOWN"])
        self.phase_label.setText(phase)
        self.phase_label.setStyleSheet(badge_style(color, radius=6))

    def update_health(self, bitmask: int):
        for name, bit in SENSOR_BITS.items():
            ok = bool(bitmask & (1 << bit))
            self.sensor_dots[name].setStyleSheet(badge_style(theme.GOOD if ok else theme.BAD))

    def update_value(self, key: str, value: float):
        # Buffered — labels repaint on the UI tick via flush_values().
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
            self._flight_labels[key].setStyleSheet(f"color:{theme.TEXT};")
        self.deploy_gauge.set_fraction(0.0)
        for dot in self.system_health_dots.values():
            dot.setStyleSheet(_off_badge())
        for dot in self.radio_ops_dots.values():
            dot.setStyleSheet(_off_badge())

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
                theme.GOOD if tol < 100 else theme.AMBER if tol < 300 else theme.BAD))
        if self._flight_speed is not None:
            speed = self._flight_labels["f_speed"]
            speed.setText(f"{self._flight_speed:.1f} m/s")
            # Green = below the mach gate (deployment allowed), amber above
            below = self._flight_speed < MACH_GATE_MPS
            speed.setStyleSheet("color:%s;" % (theme.GOOD if below else theme.AMBER))

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
                key_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
                val_label = QLabel("—")
                val_label.setFont(mono_font(9, bold=True))
                val_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                val_label.setStyleSheet(f"color:{theme.TEXT};")
                self._values_layout.addWidget(key_label, row, 0)
                self._values_layout.addWidget(val_label, row, 1)
                self._value_rows[key] = val_label
                self._dim_labels.append(key_label)
                self._val_labels.append(val_label)
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
                    bg = theme.GOOD       # green — 3D fix
                elif fix >= 0:
                    bg = theme.AMBER      # amber — online, searching/2D
                else:
                    bg = theme.BAD        # red — offline / init failed
                self.gps_fix_label.setStyleSheet(badge_style(bg, radius=3))
            except ValueError:
                pass
        elif key == "gps_sats":
            self.gps_sats_label.setText(f"{value} sats")
        elif key == "utc":
            self.gps_utc_label.setText(value)
            self.gps_utc_label.setStyleSheet(f"color:{theme.GOOD};")
        elif key == "radio":
            try:
                s = int(value)
                if s >= 0:
                    self.radio_label.setText("Si4463 OK")
                    self.radio_label.setStyleSheet(badge_style(theme.GOOD, radius=3))
                else:
                    self.radio_label.setText("OFFLINE")
                    self.radio_label.setStyleSheet(badge_style(theme.BAD, radius=3))
            except ValueError:
                pass
        elif key == "radio_rx":
            try:
                ok = int(value) >= 0
                self.radio_label.setText("RX OK" if ok else "RX BAD")
                self.radio_label.setStyleSheet(badge_style(theme.GOOD if ok else theme.AMBER, radius=3))
            except ValueError:
                pass
        elif key == "telem":
            lbl = self._link_value_labels["telem"]
            on = value.strip() == "1"
            lbl.setText("ON" if on else "OFF")
            lbl.setStyleSheet(f"color:{theme.GOOD if on else theme.TEXT_DIM};")
        elif key in self._link_value_labels:
            self._link_value_labels[key].setText(value)
