"""HORIZON theming — Space Raiders red / white / grey, dark + light.

One module owns every chrome color so the whole app re-themes from a single
switch. Access colors as module attributes (``theme.ACCENT`` etc.) so a call to
:func:`set_mode` is seen live by everything that reads them on the next paint;
widgets that bake colors in at construction call their own ``apply_theme()`` to
re-read after a switch.

Brand: the accent is Space Raiders red. Dark mode is grey-toned (default, best
for launch ops); light mode is white-toned. Multi-trace plot colors are a
separate categorical data palette (``plotgroups.py``) — chrome stays red / white
/ grey, data series stay distinguishable.
"""

from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QFont, QColor, QPainter, QPolygon, QImage, QPixmap
from PyQt5.QtWidgets import QProxyStyle, QStyle, QComboBox, QSpinBox, QDoubleSpinBox

# ─── Palettes ─────────────────────────────────────────────────────────────────
# Each mode is one flat dict; set_mode() rebinds the module-level names below so
# `theme.ACCENT` always reflects the active mode.

_PALETTES = {
    "dark": {
        "ACCENT":    "#e1251b",   # Space Raiders red — selection, focus, active
        "BG":        "#161619",   # window background (near-black grey)
        "SURFACE":   "#202024",   # raised panels, plot canvas
        "INSET":     "#121214",   # input fields, log background, gauges
        "BORDER":    "#34343c",
        "BORDER_HI": "#45454f",
        "TEXT":      "#ececed",
        "TEXT_DIM":  "#9a9aa4",
        "GOOD":      "#46b35e",
        "BAD":       "#e1251b",
        "AMBER":     "#e0a423",
        "SEL":       "#3a1e1c",   # selected-row tint (reddened)
        "HOVER":     "#2a2a30",   # button hover surface
        "BUTTON":    "#26262c",
        "TOOLBAR":   "#1b1b1f",
    },
    "light": {
        "ACCENT":    "#c2160e",   # darker red for AA contrast on white
        "BG":        "#f3f3f1",
        "SURFACE":   "#ffffff",
        "INSET":     "#ebebe8",
        "BORDER":    "#d6d6d1",
        "BORDER_HI": "#bcbcb6",
        "TEXT":      "#1b1b1e",
        "TEXT_DIM":  "#5c5c63",
        "GOOD":      "#2f8c46",
        "BAD":       "#c2160e",
        "AMBER":     "#9a6a0e",
        "SEL":       "#f6d9d6",
        "HOVER":     "#eceae6",
        "BUTTON":    "#ffffff",
        "TOOLBAR":   "#ebebe8",
    },
}

DEFAULT_MODE = "dark"
_mode = DEFAULT_MODE

# Module-level color names — rebound by set_mode(). Declared here so static
# tools see them; values are filled immediately below.
ACCENT = BG = SURFACE = INSET = BORDER = BORDER_HI = ""
TEXT = TEXT_DIM = GOOD = BAD = AMBER = SEL = HOVER = BUTTON = TOOLBAR = ""


def set_mode(mode: str) -> str:
    """Switch the active palette. Returns the resolved mode name."""
    global _mode
    if mode not in _PALETTES:
        mode = DEFAULT_MODE
    _mode = mode
    globals().update(_PALETTES[mode])
    return mode


def current_mode() -> str:
    return _mode


def is_dark() -> bool:
    return _mode == "dark"


set_mode(DEFAULT_MODE)   # initialise the module-level color names


# ─── Fonts / small helpers ────────────────────────────────────────────────────

def mono_font(size: int, bold: bool = False) -> QFont:
    f = QFont("Menlo", size, QFont.Bold if bold else QFont.Normal)
    f.setStyleHint(QFont.Monospace)
    return f


def badge_style(bg: str, fg: str = "#ffffff", radius: int = 4) -> str:
    return f"background:{bg}; color:{fg}; border-radius:{radius}px; padding:0 6px;"


# ─── Control-arrow painter (no glyph-font dependency) ─────────────────────────

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


# ─── Logos ────────────────────────────────────────────────────────────────────
# Brand artwork lives in apex/assets/. The Space Raiders wordmark is black text +
# a red swoosh on transparent — black reads on white but vanishes on grey, so for
# dark mode we recolor near-black pixels to the light text color, leaving the red
# swoosh alone. The all-red RAS mark needs no recolor (reads on both).

_ASSETS_DIR = Path(__file__).resolve().parents[2] / "assets"
LOGO_SPACE_RAIDERS = _ASSETS_DIR / "Space Raiders logo transparent.png"
LOGO_RAS = _ASSETS_DIR / "RAS logo - ttu red.png"


def _recolor_dark_pixels(img: QImage, to_hex: str) -> QImage:
    """Remap near-black opaque pixels to `to_hex`, preserving alpha and reds."""
    img = img.convertToFormat(QImage.Format_ARGB32)
    repl = QColor(to_hex)
    w, h = img.width(), img.height()
    for y in range(h):
        for x in range(w):
            px = img.pixelColor(x, y)
            if px.alpha() == 0:
                continue
            r, g, b = px.red(), px.green(), px.blue()
            # near-black (text) but not the red swoosh (r dominant)
            if r < 90 and g < 90 and b < 90:
                repl.setAlpha(px.alpha())
                img.setPixelColor(x, y, repl)
    return img


def load_logo(path: Path, height: int, recolor_dark: bool = False) -> QPixmap:
    """Load `path` scaled to `height` px tall. If recolor_dark and the active
    mode is dark, flip near-black artwork to the light text color first.
    Returns a null QPixmap if the file is missing (caller falls back to text)."""
    if not path.exists():
        return QPixmap()
    img = QImage(str(path))
    if img.isNull():
        return QPixmap()
    if recolor_dark and is_dark():
        img = _recolor_dark_pixels(img, TEXT)
    pm = QPixmap.fromImage(img)
    if pm.height() != height:
        pm = pm.scaledToHeight(height, Qt.SmoothTransformation)
    return pm


# ─── Application stylesheet ───────────────────────────────────────────────────

def build_qss() -> str:
    """The full app stylesheet for the active palette. Re-call on a mode switch
    and setStyleSheet() it on the main window."""
    return f"""
        QMainWindow, QWidget    {{ background: {BG}; color: {TEXT}; }}
        QLabel                  {{ color: {TEXT}; background: transparent; }}
        QScrollArea             {{ border: none; }}
        QSplitter::handle       {{ background: {BORDER}; width: 2px; }}
        QSplitter::handle:hover {{ background: {ACCENT}; }}
        QToolTip                {{ background: {SURFACE}; color: {TEXT};
                                   border: 1px solid {BORDER_HI}; padding: 3px 6px; }}

        /* Page tabs */
        QFrame#tabstrip         {{ background: {BG};
                                   border-bottom: 1px solid {BORDER}; }}
        QTabBar                 {{ background: transparent; }}
        QTabBar::tab            {{ background: transparent; color: {TEXT_DIM};
                                   padding: 7px 18px 5px 18px; border: none;
                                   border-bottom: 2px solid transparent;
                                   font-weight: bold; }}
        QTabBar::tab:hover      {{ color: {TEXT}; }}
        QTabBar::tab:selected   {{ color: {ACCENT};
                                   border-bottom: 2px solid {ACCENT}; }}
        QTabBar::tab:disabled   {{ color: {BORDER_HI}; }}

        /* Connection bar + toolbar */
        QFrame#connbar          {{ background: {TOOLBAR};
                                   border-bottom: 1px solid {BORDER}; }}
        QFrame#toolbar          {{ background: {SURFACE};
                                   border-bottom: 1px solid {BORDER}; }}
        QFrame#connbar QLabel,
        QFrame#toolbar QLabel   {{ color: {TEXT_DIM}; background: transparent; }}

        QFrame#connbar QComboBox, QFrame#connbar QSpinBox,
        QFrame#connbar QDoubleSpinBox, QFrame#connbar QLineEdit,
        QFrame#toolbar QComboBox, QFrame#toolbar QSpinBox,
        QFrame#toolbar QDoubleSpinBox, QFrame#toolbar QLineEdit
                                {{ background: {INSET}; border: 1px solid {BORDER};
                                   color: {TEXT}; padding: 2px 6px; border-radius: 3px; }}
        QFrame#connbar QPushButton,
        QFrame#toolbar QPushButton
                                {{ background: {BUTTON}; border: 1px solid {BORDER};
                                   color: {TEXT}; padding: 3px 10px; border-radius: 3px; }}
        QFrame#connbar QPushButton:hover,
        QFrame#toolbar QPushButton:hover
                                {{ background: {HOVER}; border-color: {ACCENT};
                                   color: {TEXT}; }}

        /* Panel controls */
        QGroupBox               {{ border: 1px solid {BORDER}; border-radius: 6px;
                                   margin-top: 7px; padding-top: 7px;
                                   color: {TEXT_DIM}; }}
        QGroupBox::title        {{ subcontrol-origin: margin; left: 10px;
                                   padding: 0 4px; color: {ACCENT}; }}

        QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit, QListWidget
                                {{ background: {INSET}; border: 1px solid {BORDER_HI};
                                   color: {TEXT}; padding: 2px 6px; border-radius: 3px; }}
        QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus,
        QLineEdit:focus, QListWidget:focus
                                {{ border-color: {ACCENT}; }}
        QListWidget::item       {{ padding: 3px 6px; }}
        QListWidget::item:selected
                                {{ background: {SEL}; color: {TEXT}; }}
        QListWidget::item:hover {{ background: {HOVER}; }}
        QPushButton             {{ background: {BUTTON}; border: 1px solid {BORDER_HI};
                                   color: {TEXT}; padding: 3px 10px; border-radius: 3px; }}
        QPushButton:hover       {{ background: {HOVER}; border-color: {ACCENT}; }}
        QPushButton:pressed     {{ background: {SEL}; }}
        QPushButton:disabled    {{ color: {BORDER_HI}; border-color: {BORDER}; }}
        QCheckBox               {{ color: {TEXT}; background: transparent; }}

        QComboBox QAbstractItemView {{
                                   background: {SURFACE}; border: 1px solid {BORDER_HI};
                                   color: {TEXT};
                                   selection-background-color: {SEL};
                                   selection-color: {TEXT}; outline: none; }}

        /* Scrollbars */
        QScrollBar:vertical     {{ background: {BG}; width: 8px; margin: 0; border: none; }}
        QScrollBar::handle:vertical
                                {{ background: {BORDER_HI}; border-radius: 4px; min-height: 24px; }}
        QScrollBar::handle:vertical:hover {{ background: {ACCENT}; }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: none; }}
        QScrollBar:horizontal   {{ background: {BG}; height: 8px; margin: 0; border: none; }}
        QScrollBar::handle:horizontal
                                {{ background: {BORDER_HI}; border-radius: 4px; min-width: 24px; }}
        QScrollBar::handle:horizontal:hover {{ background: {ACCENT}; }}
        QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
        QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{ background: none; }}
    """
