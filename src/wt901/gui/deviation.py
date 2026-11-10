"""Two-axis deviation card: φ (left/right) and ψ (up/down) as the primary readout.

The single total angle was hard to act on ("3° to where?"), so this card shows
the two orthogonal components large enough to read at a glance, plus a direction
word and a quality hint.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from . import theme
from .widgets import color_for_deviation, quality_text


def side_text(value: float, positive: str, negative: str) -> str:
    """'偏右' / '偏左' / '居中' for a signed component."""

    if abs(value) < 0.05:
        return "居中"
    return positive if value > 0 else negative


class DeviationPair(QWidget):
    """φ and ψ side by side, coloured by the total deviation."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._phi = 0.0
        self._psi = 0.0
        self._total = 0.0
        self._direction = "对准"
        self._stale = True
        self.setMinimumHeight(146)

    def set_value(
        self,
        phi: float,
        psi: float,
        total: float,
        direction: str,
        stale: bool = False,
    ) -> None:
        self._phi = phi
        self._psi = psi
        self._total = total
        self._direction = direction
        self._stale = stale
        self.update()

    # ------------------------------------------------------------------ paint
    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(1, 1, -1, -1)

        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.setBrush(QColor(theme.PANEL))
        p.drawRoundedRect(rect, 10, 10)

        color = QColor(theme.DIM if self._stale else color_for_deviation(self._total))
        mid_x = rect.center().x()

        self._column(
            p,
            QRectF(rect.left() + 6, rect.top(), rect.width() / 2 - 12, rect.height() - 26),
            "左右 φ（右+）",
            self._phi,
            "偏右",
            "偏左",
            color,
        )
        self._column(
            p,
            QRectF(mid_x + 6, rect.top(), rect.width() / 2 - 12, rect.height() - 26),
            "上下 ψ（上+）",
            self._psi,
            "偏上",
            "偏下",
            color,
        )

        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.drawLine(int(mid_x), int(rect.top() + 14), int(mid_x), int(rect.bottom() - 34))

        p.setPen(QPen(QColor(theme.DIM if self._stale else theme.MUTED)))
        f = QFont(theme.FONT_UI, 10)
        p.setFont(f)
        footer = (
            "— 无数据 —"
            if self._stale
            else f"{self._direction} · {quality_text(self._total)}"
        )
        p.drawText(
            QRectF(rect.left(), rect.bottom() - 30, rect.width(), 24),
            Qt.AlignHCenter | Qt.AlignVCenter,
            footer,
        )

    def _column(
        self,
        p: QPainter,
        area: QRectF,
        title: str,
        value: float,
        positive: str,
        negative: str,
        color: QColor,
    ) -> None:
        p.setPen(QPen(QColor(theme.MUTED)))
        f = QFont(theme.FONT_UI, 10)
        p.setFont(f)
        p.drawText(area.adjusted(0, 10, 0, 0), Qt.AlignHCenter | Qt.AlignTop, title)

        p.setPen(QPen(color))
        f = QFont(theme.FONT_MONO, 34 if abs(value) < 100.0 else 25)
        f.setBold(True)
        p.setFont(f)
        text = "—" if self._stale else f"{value:+.1f}°"
        p.drawText(area.adjusted(0, 30, 0, -26), Qt.AlignCenter, text)

        p.setPen(QPen(QColor(theme.DIM if self._stale else theme.MUTED)))
        f = QFont(theme.FONT_UI, 10)
        p.setFont(f)
        hint = "—" if self._stale else side_text(value, positive, negative)
        p.drawText(area.adjusted(0, 0, 0, -6), Qt.AlignHCenter | Qt.AlignBottom, hint)
