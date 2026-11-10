"""3D needle viewport drawn with QPainter (no OpenGL dependency).

The scene is deliberately minimal: a skin plane grid, a planned direction, the
current needle direction and the angle between them.  Everything is in
millimetres, with the needle normally pointing down.
"""

from __future__ import annotations

import math
import time
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
    def __init__(self, parent: Optional[QWidget] = None, *, compact: bool = False) -> None:
        super().__init__(parent)
        self._compact = compact
        self.setMinimumSize(220, 140) if compact else self.setMinimumSize(620, 480)
        self._axis: Vec3 = DOWN
        self._plan: Vec3 = DOWN
        self._deviation = 0.0
        self._stale = False
        self._model = None
        self._base = None
        self._model_axis = None
        self._model_hub = None
        self._model_tip = None
        self._model_pivot = None
        self._pivot_mode = "hub"
        self._model_rotation = None
        self._model_compact = None
        self._base_compact = None
        self._default_preset = "top"
        self._target = (0.0, 0.0, 0.0)
        self._preset = "top"

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
        #: Wall time of the last paintEvent, exposed in the status bar.
        self.last_paint_ms = 0.0

    # ------------------------------------------------------------------ state
    def set_pose(
        self,
        axis: Vec3,
        plan: Vec3,
        deviation_deg: float,
        stale: bool = False,
        model_rotation: Optional[Quaternion] = None,
    ) -> None:
        self._axis = axis
        self._plan = plan
        self._deviation = deviation_deg
        self._stale = stale
        # Full model -> display rotation from the tracker.  ``None`` keeps the
        # legacy shortest-arc fallback (used before the first pose arrives).
        self._model_rotation = model_rotation
        self.update()

    def set_stale(self, stale: bool) -> None:
        if stale != self._stale:
            self._stale = stale
            self.update()

    def set_model(
        self,
        mesh,
        axis=None,
        hub=None,
        tip=None,
        pivot=None,
        pivot_mode: str = "hub",
    ) -> None:
        """Needle/object model.

        ``axis``/``hub``/``tip`` are in the model's own coordinates.  The model
        rotates about ``hub`` or ``tip`` depending on ``pivot_mode``; ``pivot``
        overrides both for callers that already computed the point.  When all
        are omitted, the legacy behaviour is used: the mesh is centred and
        points down -Z.
        """

        self._model = mesh
        self._model_compact = mesh.simplified(600) if (mesh is not None and self._compact) else mesh
        self._model_axis = tuple(axis) if axis is not None else None
        self._model_hub = tuple(hub) if hub is not None else None
        self._model_tip = tuple(tip) if tip is not None else None
        self._pivot_mode = pivot_mode
        self._model_pivot = (
            tuple(pivot)
            if pivot is not None
            else self.pivot_for_mode(pivot_mode)
        )
        self._model_rotation = None
        self.update()

    def pivot_for_mode(self, mode: str):
        """Model-space pivot for ``hub`` or ``tip`` mode."""

        if mode == "tip" and self._model_tip is not None:
            return self._model_tip
        return self._model_hub

    def set_pivot_mode(self, mode: str) -> None:
        """Switch the rotation centre between the hub and the needle tip."""

        self._pivot_mode = mode
        self._model_pivot = self.pivot_for_mode(mode)
        self.update()

    def set_pivot(self, pivot, *, mode: str = "hub") -> None:
        """Set an explicit model-space rotation centre."""

        self._model_pivot = tuple(pivot)
        self._pivot_mode = mode
        self.update()

    def set_base(self, mesh) -> None:
        """Reference base (stand); drawn below the origin plane."""

        self._base = mesh
        self._base_compact = mesh.simplified(600) if (mesh is not None and self._compact) else mesh
        self.update()

    def reset_camera(self) -> None:
        self.set_camera_preset(self._default_preset)
        self._dist = 400.0
        self.update()

    def set_camera_preset(self, name: str) -> None:
        """Jump to 俯视 / 主视 / 侧视 / 等轴."""

        q = view_quaternion(name).normalize()
        self._view_q = q
        self._view_m = q.to_matrix()
        self._preset = name
        self.update()

    def set_distance(self, distance: float) -> None:
        self._dist = max(120.0, min(1600.0, distance))
        self.update()

    def fit_distance(self) -> None:
        """Frame the loaded content (or the needle) and zoom to fit."""

        meshes = [self._model]
        if not self._compact:
            meshes.append(self._base)
        meshes = [m for m in meshes if m is not None]
        if not meshes:
            sign = 1.0 if self._pivot_mode == "tip" else -1.0
            self._target = (0.0, 0.0, sign * NEEDLE_LENGTH_MM / 2.0)
            self.set_distance(2.2 * NEEDLE_LENGTH_MM + 80.0)
            return

        lows = [mesh.bounds()[0] for mesh in meshes]
        highs = [mesh.bounds()[1] for mesh in meshes]
        lo = tuple(min(v[i] for v in lows) for i in range(3))
        hi = tuple(max(v[i] for v in highs) for i in range(3))

        if (
            self._model is not None
            and self._model_axis is not None
            and self._model_pivot is not None
        ):
            q = self._model_rotation or Quaternion.from_two_vectors(self._model_axis, self._axis)
            hub = self._model_pivot
            corners = [
                (x, y, z)
                for x in (lo[0], hi[0])
                for y in (lo[1], hi[1])
                for z in (lo[2], hi[2])
            ]
            pts = [q.rotate(tuple(c[i] - hub[i] for i in range(3))) for c in corners]
            lo = tuple(min(p[i] for p in pts) for i in range(3))
            hi = tuple(max(p[i] for p in pts) for i in range(3))

        self._target = tuple((lo[i] + hi[i]) / 2.0 for i in range(3))
        extent = max(hi[i] - lo[i] for i in range(3))
        self.set_distance(2.2 * extent + 80.0)

    def set_default_preset(self, name: str) -> None:
        self._default_preset = name if name in VIEW_LABELS else "top"

    @property
    def default_preset(self) -> str:
        return self._default_preset

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
        tx, ty, tz = self._target
        px, py, pz = p[0] - tx, p[1] - ty, p[2] - tz
        x = m[0][0] * px + m[0][1] * py + m[0][2] * pz
        y = m[1][0] * px + m[1][1] * py + m[1][2] * pz
        z = m[2][0] * px + m[2][1] * py + m[2][2] * pz

        depth = self._dist - z
        if depth < 80.0:
            depth = 80.0
        focal = 1.4 * min(w, h)
        return (w / 2.0 + focal * x / depth, h / 2.0 - focal * y / depth, depth)

    # ---------------------------------------------------------------- painting
    def paintEvent(self, event) -> None:  # noqa: N802 - Qt naming
        started = time.perf_counter()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = float(self.width()), float(self.height())

        self._paint_background(p, w, h)
        self._paint_grid(p, w, h)
        base_mesh = self._base_compact if self._compact else self._base
        model_mesh = self._model_compact if self._compact else self._model
        self._paint_mesh(p, base_mesh, w, h, QColor("#4a5a6a"))
        # The model is attached to the needle.  Prefer the tracker's full
        # relative rotation (it keeps the roll about the needle); the
        # shortest-arc path is only a fallback before the first pose arrives.
        if self._model_rotation is not None:
            model_rotation = self._model_rotation
            pivot = self._model_pivot or self._model_hub or (0.0, 0.0, 0.0)
        elif self._model_axis is not None and self._model_hub is not None:
            model_rotation = Quaternion.from_two_vectors(self._model_axis, self._axis)
            pivot = self._model_pivot or self._model_hub
        else:
            model_rotation = Quaternion.from_two_vectors(DOWN, self._axis)
            pivot = (0.0, 0.0, 0.0)
        model_color = QColor(theme.DIM if self._stale else _color_for(self._deviation))
        self._paint_mesh(p, model_mesh, w, h, model_color, rotate=model_rotation, pivot=pivot)
        self._paint_reference(p, w, h)
        self._paint_planned(p, w, h)
        if not self._compact:
            self._paint_arc(p, w, h)
        self._paint_needle(p, w, h)
        if self._compact:
            self._paint_plane_overlay(p, w, h)
        else:
            self._paint_gnomon(p, w, h)
            self._paint_legend(p, w, h)

        if self._stale:
            self._paint_stale(p, w, h)
        self.last_paint_ms = (time.perf_counter() - started) * 1000.0

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
        pivot: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Flat-shaded STL mesh, painter-sorted by depth."""

        if mesh is None or not mesh.vertices or not mesh.triangles:
            return

        if rotate is None:
            verts = [tuple(v[i] - pivot[i] for i in range(3)) for v in mesh.vertices]
        else:
            verts = [
                rotate.rotate(tuple(v[i] - pivot[i] for i in range(3)))
                for v in mesh.vertices
            ]
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
            # The imported STL *is* the needle; mark the rotation centre.
            color = QColor(theme.DIM if self._stale else _color_for(self._deviation))
            entry = QRadialGradient(QPointF(a[0], a[1]), 9)
            entry.setColorAt(0.0, QColor(255, 255, 255, 220))
            entry.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 40))
            p.setBrush(entry)
            p.setPen(QPen(QColor(color), 1))
            p.drawEllipse(QPointF(a[0], a[1]), 6.0, 6.0)
            return

        # Hub mode: hub at the origin and the needle extends along +axis.
        # Tip mode: tip at the origin and the body extends along -axis.
        direction = -1.0 if self._pivot_mode == "tip" else 1.0
        tip3 = _scale(self._axis, direction * NEEDLE_LENGTH_MM)
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


    # ------------------------------------------------------- plane overlay
    _PLANE_LABELS = {
        "front": ("X", "Z"),
        "side": ("Y", "Z"),
        "top": ("X", "Y"),
    }

    def _paint_plane_overlay(self, p: QPainter, w: float, h: float) -> None:
        """2D axes plus the in-plane angle between the needle and the plan."""

        origin = self._project((0.0, 0.0, 0.0), w, h)
        ox, oy = origin[0], origin[1]

        arm = max(22.0, min(48.0, min(w, h) * 0.5 - 16.0))
        p.setPen(QPen(QColor(theme.BORDER), 1))
        p.drawLine(QPointF(ox - arm, oy), QPointF(ox + arm, oy))
        p.drawLine(QPointF(ox, oy - arm), QPointF(ox, oy + arm))

        horizontal, vertical = self._PLANE_LABELS.get(self._preset, ("X", "Y"))
        p.setPen(QPen(QColor(theme.DIM)))
        f = QFont(theme.FONT_MONO, 8)
        p.setFont(f)
        p.drawText(QPointF(min(ox + arm + 5.0, w - 12.0), oy + 4.0), horizontal)
        p.drawText(QPointF(ox - 3.0, max(oy - arm - 5.0, 10.0)), vertical)

        tip_u = self._project(_scale(self._axis, NEEDLE_LENGTH_MM), w, h)
        tip_p = self._project(_scale(self._plan, NEEDLE_LENGTH_MM), w, h)

        def screen_angle(tip) -> float:
            dx, dy = tip[0] - ox, tip[1] - oy
            return math.degrees(math.atan2(dx, -dy))  # 0 deg = up, clockwise

        def projected_length(tip) -> float:
            return math.hypot(tip[0] - ox, tip[1] - oy)

        if projected_length(tip_u) < 8.0 or projected_length(tip_p) < 8.0:
            p.setPen(QPen(QColor(theme.DIM)))
            p.drawText(QPointF(10.0, h - 10.0), "Δ —")
            return

        angle_u = screen_angle(tip_u)
        angle_p = screen_angle(tip_p)
        delta = (angle_u - angle_p + 180.0) % 360.0 - 180.0

        radius = arm * 0.72
        points = []
        steps = 16
        for i in range(steps + 1):
            a = math.radians(angle_p + delta * i / steps)
            points.append(QPointF(ox + radius * math.sin(a), oy - radius * math.cos(a)))

        color = _color_for(self._deviation)
        p.setPen(QPen(color, 1.4))
        p.drawPolyline(QPolygonF(points))

        # plan ray (dashed) and needle ray (solid)
        p.setPen(QPen(QColor(61, 220, 151, 170), 1, Qt.DashLine))
        p.drawLine(
            QPointF(ox, oy),
            QPointF(ox + radius * 1.35 * math.sin(math.radians(angle_p)),
                    oy - radius * 1.35 * math.cos(math.radians(angle_p))),
        )
        p.setPen(QPen(color, 1.6))
        p.drawLine(
            QPointF(ox, oy),
            QPointF(ox + radius * 1.35 * math.sin(math.radians(angle_u)),
                    oy - radius * 1.35 * math.cos(math.radians(angle_u))),
        )

        p.setPen(QPen(QColor(theme.TEXT)))
        f = QFont(theme.FONT_MONO, 9)
        f.setBold(True)
        p.setFont(f)
        p.drawText(QPointF(9.0, h - 9.0), f"Δ {delta:+.1f}°")

    # ------------------------------------------------------------------ mouse
    def mousePressEvent(self, event) -> None:  # noqa: N802
        if self._locked or self._compact:
            return
        self._drag = event.position()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._locked or self._compact or self._drag is None:
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
        if not self._compact:
            self.reset_camera()

    def wheelEvent(self, event) -> None:  # noqa: N802
        if self._compact:
            return
        delta = event.angleDelta().y()
        factor = 0.9 if delta > 0 else 1.1
        self._dist = max(300.0, min(1400.0, self._dist * factor))
        self.update()



# Orthographic-ish view matrices (world -> view).  Rows are the view axes
# expressed in world coordinates; the projection looks along -view_z.
VIEW_PRESETS = {
    "top": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    "front": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)),
    "side": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
}
VIEW_LABELS = {"top": "俯视", "front": "主视", "side": "侧视", "iso": "等轴"}


def view_quaternion(name: str) -> Quaternion:
    """Camera orientation for a named preset."""

    if name == "iso":
        return _default_view()
    matrix = VIEW_PRESETS.get(name, VIEW_PRESETS["top"])
    return Quaternion.from_matrix(matrix)


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
