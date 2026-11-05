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


class DeviationReadout(QWidget):
    """The number the operator actually looks at."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._deg = 0.0
        self._stale = False
        self.setMinimumHeight(132)

    def set_value(self, deviation_deg: float, stale: bool = False) -> None:
        self._deg = deviation_deg
        self._stale = stale
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.setBrush(QColor(theme.PANEL))
        p.drawRoundedRect(rect, 10, 10)

        color = QColor(theme.DIM if self._stale else color_for_deviation(self._deg))
        p.setPen(QPen(QColor(theme.MUTED)))
        f = QFont(theme.FONT_UI, 11)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1)
        p.setFont(f)
        p.drawText(rect.adjusted(18, 12, -18, 0), Qt.AlignLeft | Qt.AlignTop, "与计划偏差")

        p.setPen(QPen(color))
        f = QFont(theme.FONT_MONO, 44)
        f.setBold(True)
        p.setFont(f)
        p.drawText(rect.adjusted(0, 18, -46, 8), Qt.AlignCenter, f"{self._deg:.1f}")

        f = QFont(theme.FONT_UI, 18)
        p.setFont(f)
        p.drawText(rect.adjusted(52, 24, -18, 6), Qt.AlignCenter, "°")

        p.setPen(QPen(QColor(theme.DIM if self._stale else theme.MUTED)))
        f = QFont(theme.FONT_UI, 10)
        p.setFont(f)
        p.drawText(
            rect.adjusted(0, 0, 0, -12),
            Qt.AlignHCenter | Qt.AlignBottom,
            "— 无数据 —" if self._stale else _quality_text(self._deg),
        )


def _quality_text(deg: float) -> str:
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
