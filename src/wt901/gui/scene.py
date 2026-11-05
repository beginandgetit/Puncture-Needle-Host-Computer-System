"""3D needle viewport drawn with QPainter (no OpenGL dependency).

The scene is deliberately minimal: a skin plane grid, a planned direction, the
current needle direction and the angle between them.  Everything is in
millimetres, with the needle normally pointing down.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPen, QPolygonF, QRadialGradient
from PySide6.QtWidgets import QWidget

from . import theme
from ..pose import Quaternion, m_vec

Vec3 = Tuple[float, float, float]
DOWN: Vec3 = (0.0, 0.0, -1.0)

NEEDLE_LENGTH_MM = 170.0


class NeedleView(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(620, 480)
        self._axis: Vec3 = DOWN
        self._plan: Vec3 = DOWN
        self._deviation = 0.0
        self._stale = False
        self._model = None
        self._base = None

        # Camera orientation is a quaternion rotated about *screen* axes
        # (trackball), not a turntable around the world Z axis.  A turntable
        # would spin the grid around the vertical needle and the needle would
        # look nailed to the screen while the coordinate frame rotates -- which
        # is exactly the confusing behaviour reported.
        self._view_q = _default_view()
        self._view_m = self._view_q.to_matrix()
        self._dist = 400.0
        self._drag: Optional[QPointF] = None
        self._locked = False

    # ------------------------------------------------------------------ state
    def set_pose(self, axis: Vec3, plan: Vec3, deviation_deg: float, stale: bool = False) -> None:
        self._axis = axis
        self._plan = plan
        self._deviation = deviation_deg
        self._stale = stale
        self.update()

    def set_stale(self, stale: bool) -> None:
        if stale != self._stale:
            self._stale = stale
            self.update()

    def set_model(self, mesh) -> None:
        """Needle/object model, already positioned so the origin is inside it."""

        self._model = mesh
        self.update()

    def set_base(self, mesh) -> None:
        """Reference base (stand); drawn below the origin plane."""

        self._base = mesh
        self.update()

    def reset_camera(self) -> None:
        self._view_q = _default_view()
        self._view_m = self._view_q.to_matrix()
        self._dist = 400.0
        self.update()

    def set_view_locked(self, locked: bool) -> None:
        """Locked: the scene is a fixed picture; dragging cannot rotate it."""

        self._locked = locked
        self._drag = None
        self.setCursor(Qt.ArrowCursor if locked else Qt.OpenHandCursor)
        self.update()

    @property
    def view_locked(self) -> bool:
        return self._locked

    # ------------------------------------------------------------- projection
    def _project(self, p: Vec3, w: float, h: float) -> Tuple[float, float, float]:
        # Rigid view transform: needle and coordinate frame share it, so they
        # always rotate together when the camera turns.
        m = self._view_m
        x = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2]
        y = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2]
        z = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2]

        depth = self._dist - z
        if depth < 80.0:
            depth = 80.0
        focal = 1.4 * min(w, h)
        return (w / 2.0 + focal * x / depth, h / 2.0 - focal * y / depth, depth)

    # ---------------------------------------------------------------- painting
    def paintEvent(self, event) -> None:  # noqa: N802 - Qt naming
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = float(self.width()), float(self.height())

        self._paint_background(p, w, h)
        self._paint_grid(p, w, h)
        self._paint_mesh(p, self._base, w, h, QColor("#4a5a6a"))
        # the imported model is attached to the needle: rotate it with the pose
        model_rotation = Quaternion.from_two_vectors(DOWN, self._axis)
        model_color = QColor(theme.DIM if self._stale else _color_for(self._deviation))
        self._paint_mesh(p, self._model, w, h, model_color, rotate=model_rotation)
        self._paint_reference(p, w, h)
        self._paint_planned(p, w, h)
        self._paint_arc(p, w, h)
        self._paint_needle(p, w, h)
        self._paint_gnomon(p, w, h)
        self._paint_legend(p, w, h)

        if self._stale:
            self._paint_stale(p, w, h)

    def _paint_background(self, p: QPainter, w: float, h: float) -> None:
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0.0, QColor("#0e151d"))
        grad.setColorAt(1.0, QColor("#080d13"))
        p.fillRect(self.rect(), grad)

        glow = QRadialGradient(w * 0.5, h * 0.58, w * 0.55)
        glow.setColorAt(0.0, QColor(61, 169, 252, 26))
        glow.setColorAt(1.0, QColor(0, 0, 0, 0))
        p.fillRect(self.rect(), glow)

    def _paint_grid(self, p: QPainter, w: float, h: float) -> None:
        span, step = 200, 40
        pen = QPen(QColor(theme.GRID), 1)
        p.setPen(pen)
        for i in range(-span // step, span // step + 1):
            x = i * step
            a = self._project((x, -span, 0.0), w, h)
            b = self._project((x, span, 0.0), w, h)
            p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))
            a = self._project((-span, x, 0.0), w, h)
            b = self._project((span, x, 0.0), w, h)
            p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

    def _paint_gnomon(self, p: QPainter, w: float, h: float) -> None:
        """Small XYZ indicator fixed to the lower-right corner."""

        box = QRectF(w - 132.0, h - 122.0, 114.0, 104.0)
        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.setBrush(QColor(8, 13, 19, 165))
        p.drawRoundedRect(box, 8, 8)

        cx, cy = box.center().x(), box.center().y() + 10.0
        length = 32.0
        font = QFont(theme.FONT_MONO, 8)
        font.setBold(True)
        p.setFont(font)

        for axis, color, label in (
            ((1.0, 0.0, 0.0), QColor(255, 92, 92), "X"),
            ((0.0, 1.0, 0.0), QColor(61, 220, 151), "Y"),
            ((0.0, 0.0, 1.0), QColor(61, 169, 252), "Z"),
        ):
            v = m_vec(self._view_m, axis)
            ex, ey = cx + v[0] * length, cy - v[1] * length
            p.setPen(QPen(color, 1.8))
            p.drawLine(QPointF(cx, cy), QPointF(ex, ey))
            p.setPen(QPen(color))
            p.drawText(QPointF(ex + v[0] * 9 - 3, ey - v[1] * 9 + 4), label)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(theme.MUTED))
        p.drawEllipse(QPointF(cx, cy), 1.8, 1.8)

    def _paint_mesh(
        self,
        p: QPainter,
        mesh,
        w: float,
        h: float,
        base: QColor,
        rotate: Optional[Quaternion] = None,
    ) -> None:
        """Flat-shaded STL mesh, painter-sorted by depth."""

        if mesh is None or not mesh.vertices or not mesh.triangles:
            return

        verts = mesh.vertices if rotate is None else [rotate.rotate(v) for v in mesh.vertices]
        projected = [self._project(v, w, h) for v in verts]
        faces = []
        for ia, ib, ic in mesh.triangles:
            pa, pb, pc = projected[ia], projected[ib], projected[ic]
            depth = (pa[2] + pb[2] + pc[2]) / 3.0
            ax, ay, az = verts[ia]
            bx, by, bz = verts[ib]
            cx_, cy_, cz_ = verts[ic]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx_ - ax, cy_ - ay, cz_ - az
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            shade = abs((nx * 0.35 + ny * -0.45 + nz * 0.82) / norm)
            shade = 0.38 + 0.62 * shade
            faces.append(
                (
                    depth,
                    QPolygonF([QPointF(pa[0], pa[1]), QPointF(pb[0], pb[1]), QPointF(pc[0], pc[1])]),
                    shade,
                )
            )

        faces.sort(key=lambda item: -item[0])
        for _, poly, shade in faces:
            color = QColor(
                int(base.red() * shade), int(base.green() * shade), int(base.blue() * shade), 215
            )
            p.setBrush(color)
            p.setPen(QPen(color.darker(140), 0.7))
            p.drawPolygon(poly)

    def _paint_reference(self, p: QPainter, w: float, h: float) -> None:
        p.setPen(QPen(QColor(theme.DIM), 1, Qt.DotLine))
        a = self._project((0, 0, 0), w, h)
        b = self._project(_scale(DOWN, NEEDLE_LENGTH_MM), w, h)
        p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

    def _paint_planned(self, p: QPainter, w: float, h: float) -> None:
        pen = QPen(QColor(61, 220, 151, 190), 2, Qt.DashLine)
        pen.setDashPattern([6, 6])
        p.setPen(pen)
        a = self._project((0, 0, 0), w, h)
        b = self._project(_scale(self._plan, NEEDLE_LENGTH_MM), w, h)
        p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

    def _paint_arc(self, p: QPainter, w: float, h: float) -> None:
        if self._deviation < 0.5 or self._deviation > 100.0:
            return
        radius = 120.0
        points: List[QPointF] = []
        steps = 28
        for i in range(steps + 1):
            t = i / steps
            v = _normalize(
                (
                    self._plan[0] * (1 - t) + self._axis[0] * t,
                    self._plan[1] * (1 - t) + self._axis[1] * t,
                    self._plan[2] * (1 - t) + self._axis[2] * t,
                )
            )
            pt = self._project(_scale(v, radius), w, h)
            points.append(QPointF(pt[0], pt[1]))

        p.setPen(QPen(QColor(61, 169, 252, 170), 1.5))
        p.drawPolyline(QPolygonF(points))

    def _paint_needle(self, p: QPainter, w: float, h: float) -> None:
        base = (0.0, 0.0, 0.0)
        a = self._project(base, w, h)

        if self._model is not None:
            # The imported STL *is* the needle; just mark the entry point.
            color = QColor(theme.DIM if self._stale else _color_for(self._deviation))
            entry = QRadialGradient(QPointF(a[0], a[1]), 9)
            entry.setColorAt(0.0, QColor(255, 255, 255, 220))
            entry.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 40))
            p.setBrush(entry)
            p.setPen(QPen(QColor(color), 1))
            p.drawEllipse(QPointF(a[0], a[1]), 6.0, 6.0)
            return

        tip3 = _scale(self._axis, NEEDLE_LENGTH_MM)
        b = self._project(tip3, w, h)

        color = QColor(theme.DIM if self._stale else _color_for(self._deviation))

        glow = QColor(color)
        glow.setAlpha(45)
        pen = QPen(glow, 18)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

        pen = QPen(color, 7)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

        core = QColor(255, 255, 255, 110)
        pen = QPen(core, 2)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

        # needle tip: small cone
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        width_px = 6.0
        poly = QPolygonF(
            [
                QPointF(b[0], b[1]),
                QPointF(b[0] - ux * 16 + px * width_px, b[1] - uy * 16 + py * width_px),
                QPointF(b[0] - ux * 16 - px * width_px, b[1] - uy * 16 - py * width_px),
            ]
        )
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawPolygon(poly)

        # entry point
        entry = QRadialGradient(QPointF(a[0], a[1]), 9)
        entry.setColorAt(0.0, QColor(255, 255, 255, 220))
        entry.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 40))
        p.setBrush(entry)
        p.setPen(QPen(QColor(color), 1))
        p.drawEllipse(QPointF(a[0], a[1]), 6.5, 6.5)

    def _paint_legend(self, p: QPainter, w: float, h: float) -> None:
        x, y = 18.0, h - 46.0
        f = QFont(theme.FONT_UI, 9)
        p.setFont(f)

        p.setPen(QPen(QColor("#c9d4df"), 3))
        p.drawLine(QPointF(x, y), QPointF(x + 22, y))
        p.setPen(QPen(QColor(theme.MUTED)))
        p.drawText(QPointF(x + 30, y + 4), "当前方向")

        p.setPen(QPen(QColor(61, 220, 151), 2, Qt.DashLine))
        p.drawLine(QPointF(x + 120, y), QPointF(x + 142, y))
        p.setPen(QPen(QColor(theme.MUTED)))
        p.drawText(QPointF(x + 150, y + 4), "计划方向")

        p.setPen(QPen(QColor(theme.DIM), 1))
        hint = (
            "视角已锁定 · 滚轮缩放 · 勾选「自由视角」后才能旋转"
            if self._locked
            else "拖动旋转 · 滚轮缩放 · 双击复位"
        )
        p.drawText(QPointF(x, y + 18), hint)

        if self._locked:
            p.setPen(QPen(QColor(theme.DIM), 1))
            p.drawText(QPointF(w - 74, 24), "视角锁定")

    def _paint_stale(self, p: QPainter, w: float, h: float) -> None:
        p.fillRect(self.rect(), QColor(8, 13, 19, 205))
        p.setPen(QPen(QColor(theme.BAD)))
        f = QFont(theme.FONT_UI, 26)
        f.setBold(True)
        p.setFont(f)
        p.drawText(QRectF(0, h / 2 - 46, w, 46), Qt.AlignCenter, "NO DATA")
        p.setPen(QPen(QColor(theme.MUTED)))
        f = QFont(theme.FONT_UI, 11)
        p.setFont(f)
        p.drawText(QRectF(0, h / 2 + 2, w, 24), Qt.AlignCenter, "等待传感器数据…")

    # ------------------------------------------------------------------ mouse
    def mousePressEvent(self, event) -> None:  # noqa: N802
        if self._locked:
            return
        self._drag = event.position()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._locked or self._drag is None:
            return
        pos = event.position()
        dx = pos.x() - self._drag.x()
        dy = pos.y() - self._drag.y()
        # Rotate about the *screen* axes (trackball).  The needle and the
        # coordinate frame are transformed by the same rigid rotation, so they
        # visibly turn together instead of the needle staying pinned.
        q_yaw = Quaternion.from_axis_angle((0.0, 1.0, 0.0), dx * 0.4)
        q_pitch = Quaternion.from_axis_angle((1.0, 0.0, 0.0), dy * 0.3)
        self._apply_view(q_pitch * q_yaw * self._view_q)
        self._drag = pos

    def _apply_view(self, q: Quaternion) -> None:
        self._view_q = q.normalize()
        self._view_m = self._view_q.to_matrix()
        self.update()

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        self._drag = None

    def mouseDoubleClickEvent(self, event) -> None:  # noqa: N802
        self.reset_camera()

    def wheelEvent(self, event) -> None:  # noqa: N802
        delta = event.angleDelta().y()
        factor = 0.9 if delta > 0 else 1.1
        self._dist = max(300.0, min(1400.0, self._dist * factor))
        self.update()


# ----------------------------------------------------------------- helpers
def _default_view() -> Quaternion:
    """Isometric-ish starting view: 34 deg around Z, 28 deg tilt."""

    return Quaternion.from_axis_angle((1.0, 0.0, 0.0), -28.0) * Quaternion.from_axis_angle(
        (0.0, 0.0, 1.0), 34.0
    )


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _normalize(v: Vec3) -> Vec3:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _color_for(deviation: float) -> QColor:
    if deviation < 2.0:
        return QColor(theme.OK)
    if deviation < 5.0:
        return QColor(theme.WARN)
    return QColor(theme.BAD)
