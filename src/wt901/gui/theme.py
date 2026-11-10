"""Visual constants for the prototype UI."""

from __future__ import annotations

BG = "#0b1118"
PANEL = "#121a24"
PANEL_HI = "#18232f"
BORDER = "#22303f"
GRID = "#1b2836"

TEXT = "#e7eef5"
MUTED = "#8c9aa8"
DIM = "#55636f"

ACCENT = "#3da9fc"
OK = "#3ddc97"
WARN = "#f4b942"
BAD = "#ff5c5c"

FONT_UI = "Microsoft YaHei UI"
FONT_MONO = "Consolas"

def stylesheet() -> str:
    """Built lazily so the caller can register a font family first."""

    return f"""
QWidget {{
    background: {BG};
    color: {TEXT};
    font-family: "{FONT_UI}";
    font-size: 13px;
}}
QFrame#Card {{
    background: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
QLabel {{
    background: transparent;
}}
QLabel#PanelTitle {{
    color: {MUTED};
    font-size: 12px;
    letter-spacing: 1px;
}}
QLabel#Banner {{
    background: {BAD};
    color: #10161d;
    font-weight: bold;
    padding: 7px 12px;
    border-radius: 6px;
}}
QLabel#BannerWarn {{
    background: {WARN};
    color: #10161d;
    font-weight: bold;
    padding: 7px 12px;
    border-radius: 6px;
}}
QPushButton {{
    background: {PANEL_HI};
    border: 1px solid {BORDER};
    border-radius: 7px;
    padding: 8px 14px;
    color: {TEXT};
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed {{ background: #0f1720; }}
QPushButton#Primary {{
    background: {ACCENT};
    color: #06121d;
    font-weight: bold;
    border: none;
}}
QPushButton#Primary:hover {{ background: #5bb8ff; }}
QStatusBar {{ background: {PANEL}; color: {MUTED}; }}
QPushButton:checked {{ background: {ACCENT}; color: #06121d; border: 1px solid {ACCENT}; }}
QComboBox {{ background: {PANEL_HI}; border: 1px solid {BORDER}; border-radius: 6px; padding: 4px 8px; color: {TEXT}; }}
QComboBox QAbstractItemView {{ background: {PANEL}; color: {TEXT}; selection-background-color: {ACCENT}; }}
"""


STYLE_SHEET = stylesheet()
