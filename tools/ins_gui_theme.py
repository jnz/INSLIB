"""Dark/light colour themes for inspostgui.py and ins_map_view.py.

Same scheme as inslib_gui.py: two palettes, every colour of the
window comes out of whichever one is current, and whatever fixes a colour
at build time (an inline stylesheet, a pen, a GL item) registers a
callable with themed() that set_theme() runs again after a switch.
"""

import pyqtgraph as pg
from PyQt6 import QtCore

DARK = {
    "bg": "#0f1216", "panel": "#171b21", "raised": "#1f2832",
    "edge": "#252c35", "line": "#2b3642", "field": "#12151a",
    "hover": "#283441", "box_edge": "#3a4654", "strong": "#ffffff",
    "text": "#d8dee6", "dim": "#8fa0b4", "idle": "#5a6470",
    "accent": "#54aaff",
    # Plot traces: N/E/D (also x/y/z, roll/pitch/yaw), reference, 1-sigma
    # band, warm-up marker, a fourth series.
    "trace": ("#ff6a6a", "#5af08a", "#54aaff"),
    "ref": "#e8e8e8", "sigma": "#b0b0b0", "warm": "#ffb454",
    "extra": "#c080ff", "est": "#64c8ff",
    # Map tab and 3D view overlays.
    "map_est": "#ff6a6a", "map_here": "#ffb454", "fix": "#ffc83c",
    # The previous run, drawn behind the current one.
    "ghost": "#9aa4b2",
    # Where a ZUPT/ZARU was applied, on the 3D trail.
    "zupt": "#ff5ad2",
    # Ranging anchors, once ranges are fed to the filter.
    "anchor": "#ff9f43",
    "gl_bg": "#121418", "gl_grid": "#3c414b", "ref_trail": "#8cf08c",
    "position": "#ff5a5a", "ellipsoid": "#64c8ff",
    # Accelerometer bubble.
    "bubble_ring": "#505a64", "bubble_edge": "#ffdc64",
    "bubble_fill": "#ffb43c",
    # Brightness of the speed-coloured trail (HSV value).
    "trail_value": 1.0,
}

# Not the dark one inverted: traces that read well on near-black are glare
# on white, so they are darkened, and the greys are picked for contrast
# against paper.
LIGHT = {
    "bg": "#fbfbfd", "panel": "#eceff4", "raised": "#e2e7ee",
    "edge": "#ccd3dd", "line": "#b6bfcc", "field": "#ffffff",
    "hover": "#d6dde7", "box_edge": "#9aa4b2", "strong": "#101418",
    "text": "#1b2027", "dim": "#5b6675", "idle": "#98a2b0",
    "accent": "#1f6fb2",
    "trace": ("#c0392b", "#1c8a4e", "#1f6fb2"),
    "ref": "#303840", "sigma": "#7a8490", "warm": "#b86e00",
    "extra": "#7d3c98", "est": "#1f6fb2",
    "map_est": "#c0392b", "map_here": "#b86e00", "fix": "#c88a00",
    "ghost": "#8a93a0",
    "zupt": "#c2188f",
    "anchor": "#d35400",
    "gl_bg": "#fbfbfd", "gl_grid": "#b9c0ca", "ref_trail": "#1c8a4e",
    "position": "#c0392b", "ellipsoid": "#1f6fb2",
    "bubble_ring": "#9aa4b2", "bubble_edge": "#b86e00",
    "bubble_fill": "#e0a030",
    "trail_value": 0.75,
}

THEMES = {"dark": DARK, "light": LIGHT}

# The palette in force. Read at CALL time everywhere (theme.T["x"]), never
# copied into a constant at import, so a switch reaches the whole window.
T = DARK

_THEMED = []


def themed(fn):
    """Run fn now, and again after every theme change."""
    _THEMED.append(fn)
    fn()
    return fn


def set_theme(name, app=None):
    """Switch palette and repaint everything that registered a callable."""
    global T
    T = THEMES.get(name, DARK)
    pg.setConfigOptions(background=T["bg"], foreground=T["dim"])
    if app is not None:
        app.setStyleSheet(stylesheet())
    for fn in list(_THEMED):
        try:
            fn()
        except RuntimeError:
            # The Qt object behind it has been deleted.
            _THEMED.remove(fn)


def name():
    return "light" if T is LIGHT else "dark"


def dim():
    """The caption colour, as a stylesheet fragment."""
    return "color: %s;" % T["dim"]


def stylesheet():
    return (
        "QMainWindow, QWidget { background: %(bg)s; color: %(text)s; } "
        "QFrame#panel { background: %(panel)s; border: 1px solid %(edge)s; "
        "border-radius: 6px; } "
        "QGroupBox { border: 1px solid %(edge)s; border-radius: 6px; "
        "margin-top: 8px; padding-top: 12px; } "
        "QGroupBox::title { color: %(dim)s; left: 8px; } "
        "QPushButton { background: %(raised)s; border: 1px solid %(line)s; "
        "border-radius: 4px; padding: 6px 12px; color: %(text)s; } "
        "QPushButton:hover { background: %(hover)s; } "
        "QPushButton:checked { background: %(hover)s; } "
        "QPushButton:disabled { color: %(idle)s; } "
        "QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit { "
        "background: %(field)s; border: 1px solid %(line)s; padding: 3px; "
        "color: %(text)s; } "
        "QComboBox:disabled { color: %(idle)s; } "
        "QCheckBox::indicator { width: 13px; height: 13px; border: 1px solid "
        "%(box_edge)s; border-radius: 3px; background: %(field)s; } "
        "QCheckBox::indicator:checked { background: %(accent)s; } "
        "QTabWidget::pane { border: 1px solid %(edge)s; } "
        "QTabBar::tab { background: %(panel)s; padding: 6px 14px; } "
        "QTabBar::tab:selected { background: %(hover)s; color: %(strong)s; }"
    ) % T


def pen(key, width=1.0, style=None, index=None):
    """A pen out of the palette. `index` picks from a tuple entry (trace)."""
    colour = T[key] if index is None else T[key][index % len(T[key])]
    p = pg.mkPen(colour, width=width)
    if style is not None:
        p.setStyle(style)
    return p


DASH = QtCore.Qt.PenStyle.DashLine
DOT = QtCore.Qt.PenStyle.DotLine


def rgba(key, alpha=1.0):
    """Palette colour as a 0..1 float RGBA tuple, for the GL items."""
    c = pg.mkColor(T[key])
    return (c.redF(), c.greenF(), c.blueF(), alpha)


def style_plot(p):
    """Put a plot's frame (background, axes, labels, title, legend) on
    theme. `p` is a PlotWidget or a PlotItem."""
    item = p.getPlotItem() if hasattr(p, "getPlotItem") else p
    if hasattr(p, "setBackground"):
        p.setBackground(T["bg"])
    fg = pg.mkColor(T["dim"])
    for side in ("left", "bottom", "right", "top"):
        ax = item.getAxis(side)
        ax.setPen(fg)
        ax.setTextPen(fg)
        if ax.label is not None and ax.labelText:
            ax.setLabel(ax.labelText, units=ax.labelUnits, color=T["dim"])
    if item.titleLabel is not None and item.titleLabel.text:
        item.setTitle(item.titleLabel.text, color=T["dim"])
    lg = item.legend
    if lg is not None:
        lg.setLabelTextColor(fg)
