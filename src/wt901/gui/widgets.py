"""Small painted widgets: deviation readout, bullseye, value rows."""

from __future__ import annotations


from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QHBoxLayout, QLabel, QWidget

from . import theme


def color_for_deviation(deg: float) -> str:
    if deg < 2.0:
        return theme.OK
    if deg < 5.0:
        return theme.WARN
    return theme.BAD


def quality_text(deg: float) -> str:
    if deg < 2.0:
        return "对准良好"
    if deg < 5.0:
        return "接近计划方向"
    return "偏离计划方向"


class ValueRow(QWidget):
    def __init__(self, label: str, unit: str = "", parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        self._label = QLabel(label)
        self._label.setStyleSheet(f"color:{theme.MUTED};")
        self._value = QLabel("—")
        self._value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._value.setStyleSheet(
            f"color:{theme.TEXT}; font-family:'{theme.FONT_MONO}'; font-size:14px;"
        )
        self._unit = QLabel(unit)
        self._unit.setStyleSheet(f"color:{theme.DIM};")
        layout.addWidget(self._label)
        layout.addStretch(1)
        layout.addWidget(self._value)
        layout.addWidget(self._unit)

    def set_value(self, value: float | str, digits: int = 1) -> None:
        if isinstance(value, str):
            self._value.setText(value)
        else:
            self._value.setText(f"{value:.{digits}f}")

    def set_color(self, color: str) -> None:
        self._value.setStyleSheet(
            f"color:{color}; font-family:'{theme.FONT_MONO}'; font-size:14px;"
        )

    def clear(self) -> None:
        self._value.setText("—")
