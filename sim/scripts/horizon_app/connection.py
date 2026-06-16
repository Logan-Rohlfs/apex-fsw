"""Persistent, adaptive connection bar — the FC / SDR link, lifted out of the
per-page toolbars so it reads as one device-connection concern across pages.

Single adaptive control: it shows the device parameters the active page needs
(USB serial for Sensors/Logs, the SDR for Radio, fake/real FC for HIL) plus one
Connect button and a live status readout. HIL is the exception the user asked
for — it has no Connect here; its run is started from the page toolbar's Start
button, so this bar only chooses what HIL runs against.

The bar owns its widgets and exposes them as attributes; MainWindow reads their
values when starting a worker and wires their change signals. Branding (the RAS
parent-org mark and the Space Raiders division logo) lives here too.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QComboBox, QPushButton, QLineEdit,
    QSpinBox, QDoubleSpinBox, QCheckBox,
)

from radio_gfsk_rx import DEFAULT_FREQ_HZ, DEFAULT_SAMPLE_RATE_HZ

from . import theme
from .theme import mono_font, load_logo, LOGO_RAS, LOGO_SPACE_RAIDERS

_DOT = 9   # status dot diameter


class ConnectionBar(QFrame):
    connect_clicked      = pyqtSignal()
    refresh_clicked      = pyqtSignal()
    theme_toggle_clicked = pyqtSignal()
    radio_param_changed  = pyqtSignal()
    hil_fake_toggled     = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setObjectName("connbar")
        self.setFixedHeight(48)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 0, 10, 0)
        lay.setSpacing(8)

        # ── Branding: RAS parent-org mark + product wordmark ──────────────────
        self.ras_mark = QLabel()
        self.ras_mark.setFixedWidth(26)
        self.ras_mark.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.ras_mark)

        wm = QLabel("HORIZON")
        wm.setFont(mono_font(13, bold=True))
        self.wordmark = wm
        sub = QLabel("APEX GROUND")
        sub.setFont(mono_font(8))
        self.wordmark_sub = sub
        brand = QHBoxLayout()
        brand.setSpacing(6)
        brand.addWidget(wm)
        brand.addWidget(sub)
        lay.addLayout(brand)

        self._divider1 = self._vline()
        lay.addWidget(self._divider1)

        # ── Source status ─────────────────────────────────────────────────────
        src_cap = QLabel("SOURCE")
        src_cap.setFont(mono_font(8))
        self._src_cap = src_cap
        lay.addWidget(src_cap)

        self.source_name = QLabel("—")
        self.source_name.setFont(mono_font(9, bold=True))
        lay.addWidget(self.source_name)

        self.status_dot = QLabel()
        self.status_dot.setFixedSize(_DOT, _DOT)
        lay.addWidget(self.status_dot)

        self.status_label = QLabel("Not connected")
        self.status_label.setFont(mono_font(9))
        lay.addWidget(self.status_label)

        lay.addStretch()

        # ── Device controls (shown/hidden by set_mode) ────────────────────────
        # Serial / Logs / HIL-on-hardware: USB port + baud.
        self.port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedWidth(28)
        self.refresh_btn.setToolTip("Rescan serial ports")
        self.refresh_btn.clicked.connect(self.refresh_clicked)
        self.baud_label = QLabel("Baud:")
        self.baud_combo = QComboBox()
        for b in ["9600", "57600", "115200", "230400", "460800", "921600"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("921600")
        for w in (self.port_label, self.port_combo, self.refresh_btn,
                  self.baud_label, self.baud_combo):
            lay.addWidget(w)

        # Radio (RTL-SDR) device config — connection-level; retunes live.
        self.radio_freq_label = QLabel("Freq:")
        self.radio_freq_spin = QDoubleSpinBox()
        self.radio_freq_spin.setRange(100.0, 1000.0)
        self.radio_freq_spin.setDecimals(3)
        self.radio_freq_spin.setSingleStep(0.001)
        self.radio_freq_spin.setValue(DEFAULT_FREQ_HZ / 1e6)
        self.radio_freq_spin.setSuffix(" MHz")
        self.radio_freq_spin.setFixedWidth(104)
        self.radio_gain_label = QLabel("Gain:")
        self.radio_gain_input = QLineEdit("10")
        self.radio_gain_input.setFixedWidth(46)
        self.radio_gain_input.setToolTip(
            'RTL tuner gain in dB, or "auto".\n'
            'Note: rtl_sdr treats 0 as "enable tuner AGC" (≈ max gain), not 0 dB.')
        self.radio_ppm_label = QLabel("PPM:")
        self.radio_ppm_spin = QSpinBox()
        self.radio_ppm_spin.setRange(-200, 200)
        self.radio_ppm_spin.setValue(0)
        self.radio_ppm_spin.setFixedWidth(62)
        self.radio_rate_label = QLabel("Rate:")
        self.radio_rate_spin = QSpinBox()
        self.radio_rate_spin.setRange(48000, 2400000)
        self.radio_rate_spin.setSingleStep(48000)
        self.radio_rate_spin.setValue(DEFAULT_SAMPLE_RATE_HZ)
        self.radio_rate_spin.setFixedWidth(92)
        self.radio_device_label = QLabel("Dev:")
        self.radio_device_input = QLineEdit("0")
        self.radio_device_input.setFixedWidth(36)
        self._radio_widgets = [
            self.radio_freq_label, self.radio_freq_spin,
            self.radio_gain_label, self.radio_gain_input,
            self.radio_ppm_label, self.radio_ppm_spin,
            self.radio_rate_label, self.radio_rate_spin,
            self.radio_device_label, self.radio_device_input,
        ]
        for w in self._radio_widgets:
            lay.addWidget(w)
        for sig in (self.radio_freq_spin.valueChanged, self.radio_ppm_spin.valueChanged,
                    self.radio_rate_spin.valueChanged):
            sig.connect(self.radio_param_changed)
        self.radio_gain_input.editingFinished.connect(self.radio_param_changed)
        self.radio_device_input.editingFinished.connect(self.radio_param_changed)

        # HIL: choose what to run against (the run itself starts from the toolbar)
        self.hil_fake_check = QCheckBox("Fake FC")
        self.hil_fake_check.setToolTip(
            "Run against the in-process reference flight computer\n"
            "(apex_sim.hil.fake_teensy) instead of a real Teensy.")
        self.hil_fake_check.setChecked(True)
        self.hil_fake_check.toggled.connect(self.hil_fake_toggled)
        lay.addWidget(self.hil_fake_check)
        self.hil_hint = QLabel("use  ▶ Start run")
        self.hil_hint.setFont(mono_font(9))
        lay.addWidget(self.hil_hint)

        # ── Connect + theme toggle + division logo ────────────────────────────
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setFixedWidth(96)
        self.connect_btn.clicked.connect(self.connect_clicked)
        lay.addWidget(self.connect_btn)

        self._divider2 = self._vline()
        lay.addWidget(self._divider2)

        self.theme_btn = QPushButton()
        self.theme_btn.setFixedWidth(30)
        self.theme_btn.setToolTip("Toggle light / dark theme")
        self.theme_btn.clicked.connect(self.theme_toggle_clicked)
        lay.addWidget(self.theme_btn)

        self.sr_logo = QLabel()
        self.sr_logo.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        lay.addWidget(self.sr_logo)

        self._connected = False
        self.apply_theme()

    @staticmethod
    def _vline() -> QFrame:
        ln = QFrame()
        ln.setFrameShape(QFrame.VLine)
        ln.setFixedWidth(1)
        return ln

    # ── Theming ───────────────────────────────────────────────────────────────
    def apply_theme(self):
        self.wordmark.setStyleSheet(f"color:{theme.ACCENT}; background:transparent;")
        self.wordmark_sub.setStyleSheet(f"color:{theme.TEXT_DIM}; background:transparent;")
        self._src_cap.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self.hil_hint.setStyleSheet(f"color:{theme.TEXT_DIM};")
        for ln in (self._divider1, self._divider2):
            ln.setStyleSheet(f"background:{theme.BORDER};")
        self.theme_btn.setText("☾" if theme.is_dark() else "☀")
        self.ras_mark.setPixmap(load_logo(LOGO_RAS, 22))
        self.sr_logo.setPixmap(load_logo(LOGO_SPACE_RAIDERS, 18, recolor_dark=True))
        if self.ras_mark.pixmap() is None or self.ras_mark.pixmap().isNull():
            self.ras_mark.setText("◑")
            self.ras_mark.setStyleSheet(f"color:{theme.ACCENT}; font-size:18px;")
        self._restyle_status()

    # ── State ───────────────────────────────────────────────────────────────
    def set_mode(self, mode: str, hil_fake: bool):
        serial = mode in ("serial", "logs")
        radio = mode == "radio"
        hil = mode == "hil"
        hil_real = hil and not hil_fake

        for w in (self.port_label, self.port_combo, self.refresh_btn):
            w.setVisible(serial or hil_real)
        for w in (self.baud_label, self.baud_combo):
            w.setVisible(serial)
        for w in self._radio_widgets:
            w.setVisible(radio)
        self.hil_fake_check.setVisible(hil)
        self.hil_hint.setVisible(hil)
        # HIL starts from the toolbar; everything else connects here.
        self.connect_btn.setVisible(not hil)

        names = {"serial": "APEX FC", "logs": "APEX FC",
                 "radio": "RTL-SDR", "hil": "HIL"}
        self.source_name.setText(names.get(mode, "—"))

    def set_status(self, connected: bool, text: str):
        self._connected = connected
        self.status_label.setText(text)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self._restyle_status()

    def _restyle_status(self):
        on = self._connected
        self.status_dot.setStyleSheet(
            f"background:{theme.GOOD if on else theme.BORDER_HI};"
            f" border-radius:{_DOT // 2}px;")
        self.status_label.setStyleSheet(
            f"color:{theme.GOOD if on else theme.TEXT_DIM};")
        if on:
            self.connect_btn.setStyleSheet(
                f"background:{theme.BAD}; color:#ffffff; border:1px solid {theme.BAD};")
        else:
            self.connect_btn.setStyleSheet("")
