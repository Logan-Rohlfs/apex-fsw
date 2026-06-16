"""Reusable plot/display widgets: spectrum + waterfall, plot groups, gauge.

Ported from horizon.py with colors routed through `theme` and an `apply_theme()`
on the custom-painted widgets so a light/dark switch re-skins them live. The
plot *data* traces keep their categorical hues (constants.py) — only chrome
follows the theme.
"""

from __future__ import annotations

import time
from collections import deque

import numpy as np
import pyqtgraph as pg

from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox,
    QLineEdit, QCompleter,
)

from radio_gfsk_rx import DEFAULT_FREQ_HZ, DEFAULT_SAMPLE_RATE_HZ

from . import theme
from .theme import mono_font
from .constants import (
    MAX_POINTS, DEFAULT_WINDOW_S, FFT_BINS, WATERFALL_ROWS,
    EXPECTED_TX_HZ, EXPECTED_TX_BW_HZ,
)

# Fixed data-annotation color for the expected-TX overlay — intentionally not
# the brand accent, so the "where the FC should transmit" marker stays distinct
# from chrome in both themes.
TX_MARKER_COLOR = "#00d4ff"


def _rgba(hex6: str, alpha: int) -> QColor:
    c = QColor(hex6)
    c.setAlpha(alpha)
    return c


# ─── Plot widget — passes wheel events up so the scroll area scrolls ─────────

class PlotWidget(pg.PlotWidget):
    """PlotWidget that ignores wheel events so the parent QScrollArea can scroll."""
    def wheelEvent(self, ev):
        ev.ignore()


class CommandInput(QLineEdit):
    """Line edit with terminal-style command history and Tab completion."""
    def __init__(self, commands: list, parent=None):
        super().__init__(parent)
        self._commands = sorted(commands)
        self._history: list = []
        self._history_index = None
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
        ctrl_row.addWidget(self.range_label)
        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        # Spectrum plot
        self.spec_plot = PlotWidget(background=theme.SURFACE)
        self.spec_plot.setFixedHeight(170)
        self.spec_plot.showGrid(x=True, y=True, alpha=0.25)
        self.spec_plot.setMouseEnabled(x=False, y=False)
        self.spec_plot.hideButtons()
        self.spec_plot.getAxis("left").setLabel("dBFS")
        self.spec_curve = self.spec_plot.plot()
        self.peak_curve = self.spec_plot.plot(pen=pg.mkPen("#ff6b35", width=1.0))
        layout.addWidget(self.spec_plot)

        # Waterfall
        self.wf_plot = PlotWidget(background=theme.SURFACE)
        self.wf_plot.setFixedHeight(220)
        self.wf_plot.setMouseEnabled(x=False, y=False)
        self.wf_plot.hideButtons()
        self.wf_plot.hideAxis("left")
        self.wf_plot.invertY(True)   # newest row at top, history flows downward
        self.wf_plot.getAxis("bottom").setLabel("MHz")
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
                brush=pg.mkBrush(TX_MARKER_COLOR + "20"),
                pen=pg.mkPen(TX_MARKER_COLOR + "50"))
            region.setZValue(10)
            plot.addItem(region)
            line = pg.InfiniteLine(
                pos=tx_c, angle=90, movable=False,
                pen=pg.mkPen(TX_MARKER_COLOR, width=1, style=Qt.DashLine))
            line.setZValue(11)
            plot.addItem(line)

        self.tx_label = QLabel(
            f"expected TX {tx_c:.3f} MHz ±{EXPECTED_TX_BW_HZ / 2e3:.0f} kHz")
        self.tx_label.setFont(mono_font(8))
        ctrl_row.insertWidget(ctrl_row.count() - 1, self.tx_label)

        self.apply_theme()
        self.set_params(DEFAULT_FREQ_HZ, DEFAULT_SAMPLE_RATE_HZ)

    def apply_theme(self):
        self.range_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self.tx_label.setStyleSheet(f"color:{TX_MARKER_COLOR};")
        self.spec_curve.setPen(pg.mkPen(theme.ACCENT, width=1.2))
        for plot in (self.spec_plot, self.wf_plot):
            plot.setBackground(theme.SURFACE)
            for side in ("bottom", "left"):
                axis = plot.getAxis(side)
                axis.setPen(pg.mkPen(theme.BORDER_HI))
                axis.setTextPen(pg.mkPen(theme.TEXT_DIM))

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
        self._dirty = set()                # keys with data since last refresh
        self._last_window = window_s
        self._was_offscreen = False        # skipped while scrolled out / on another page
        self._event_lines = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.plot = PlotWidget(background=theme.SURFACE)
        self.plot.setMinimumHeight(160)
        # Bound the height: raster cost scales with pixel area, so fullscreen
        # should show more plots, not gigantic ones.
        self.plot.setMaximumHeight(260)
        self.plot.setAntialiasing(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.getAxis("bottom").setLabel("t (s)")
        # y auto-ranges; x is scrolled by refresh() against wall-clock time so
        # the view glides at the UI frame rate even when data arrives in bursts
        self.plot.enableAutoRange(axis="y")
        self._last_push = None   # monotonic time of newest sample
        self._legend = self.plot.addLegend(offset=(-10, 10))

        self.curves = {}
        for i, key in enumerate(keys):
            self._make_curve(key, colors[i % len(colors)])

        layout.addWidget(self.plot)
        self.apply_theme()

    def apply_theme(self):
        self.plot.setBackground(theme.SURFACE)
        for side in ("bottom", "left"):
            axis = self.plot.getAxis(side)
            axis.setPen(pg.mkPen(theme.BORDER_HI))
            axis.setTextPen(pg.mkPen(theme.TEXT_DIM))
        try:
            self._legend.setBrush(pg.mkBrush(_rgba(theme.SURFACE, 210)))
            self._legend.setPen(pg.mkPen(theme.BORDER))
            self._legend.setLabelTextColor(theme.TEXT)
        except Exception:
            pass

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
                "fill": pg.mkBrush(_rgba(theme.SURFACE, 204)),
                "movable": False,
            },
        )
        line.setZValue(20)
        self.plot.addItem(line)
        self._event_lines.append(line)

    def add_key(self, key: str, color: str = None):
        """Dynamically add a key not in the original definition."""
        if key in self._t:
            return
        color = color or theme.TEXT
        self.keys.append(key)
        self.colors.append(color)
        self._t[key] = deque(maxlen=MAX_POINTS)
        self._y[key] = deque(maxlen=MAX_POINTS)
        self._make_curve(key, color)


# ─── Deployment gauge ────────────────────────────────────────────────────────

class DeploymentGauge(QWidget):
    """Horizontal 0–100 % bar for airbrake deployment.

    Driven from the `deployment` numeric key (0.0–1.0), so it works for the
    HIL source and the radio telemetry downlink alike. Reads `theme` live in
    paintEvent, so a mode switch needs only an update().
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

    def apply_theme(self):
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        r = self.rect().adjusted(1, 1, -1, -1)

        p.setPen(pg.mkPen(theme.BORDER_HI))
        p.setBrush(QColor(theme.INSET))
        p.drawRoundedRect(r, 4, 4)

        fill = QRectF(r)
        fill.setWidth(r.width() * self._frac)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.ACCENT))
        p.save()
        p.setClipRect(fill)
        p.drawRoundedRect(QRectF(r), 4, 4)
        p.restore()

        p.setFont(mono_font(10, bold=True))
        p.setPen(QColor("#ffffff" if self._frac > 0.55 else theme.TEXT))
        p.drawText(r, Qt.AlignCenter, f"{self._frac * 100:.1f}%")
        p.end()
