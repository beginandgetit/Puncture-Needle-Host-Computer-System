"""Needle pose tracking: device quaternion -> needle axis -> model pose.

Scope (confirmed): **angle guidance only**.  No position, no insertion depth,
no external tracking.

Conventions
-----------
``s``   needle axis expressed in the *sensor* frame (constant after mounting;
        default assumes the sensor +Z is parallel to the needle).
``q``   sensor -> world rotation.  The device quaternion is used directly when
        available; Euler angles are only a fallback for old firmware.
``q0``  reference pose, captured by the operator.  Every displayed angle is
        relative to it, so magnetic north is never needed: the hospital room is
        full of steel and the magnetometer cannot be trusted.

Display frame
-------------
The display frame is frozen at the reference pose: its Z axis points *up*
(opposite the reference needle), so the needle reads straight down at capture.

``NeedlePose.quat`` is the rotation since the reference, expressed in that
display frame (identity at reference).  ``NeedlePose.model_rotation`` is the
full model -> display rotation: ``quat * canonical_placement(model_axis)``.
The renderer must use ``model_rotation`` rather than re-deriving a shortest-arc
axis alignment, otherwise the roll about the needle is lost and a handle/PCB
model no longer follows the real assembly.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .mapping import (
    canonical_placement,
    display_matrix,
    display_relative_rotation,
    quaternion_from_reading,
)
from .mapping import calibrate_axis_from_poses as _calibrate_two_pose
from .mapping import calibrate_axis_vertical as _calibrate_vertical
from .pose import (
    Mat3,
    Quaternion,
    Vec3,
    v_angle_deg,
    v_cross,
    v_dot,
    v_norm,
    v_normalize,
    v_scale,
    v_sub,
)

DOWN: Vec3 = (0.0, 0.0, -1.0)
_X: Vec3 = (1.0, 0.0, 0.0)
_Y: Vec3 = (0.0, 1.0, 0.0)
_Z: Vec3 = (0.0, 0.0, 1.0)


def plan_frame(plan: Vec3) -> Tuple[Vec3, Vec3]:
    """Right / up unit vectors perpendicular to the planned direction.

    "Up" is gravity referenced so ψ means what an operator expects (above or
    below the plan).  When the plan is (nearly) vertical that reference is
    degenerate, so the display Y axis is used instead.
    """

    p = v_normalize(plan)
    up = v_sub(_Z, v_scale(p, v_dot(_Z, p)))
    if v_norm(up) < 0.15:
        up = v_sub(_Y, v_scale(p, v_dot(_Y, p)))
    if v_norm(up) < 0.15:
        up = v_sub(_X, v_scale(p, v_dot(_X, p)))
    up = v_normalize(up)
    right = v_normalize(v_cross(p, up))
    return right, up


def plan_deviation(axis: Vec3, plan: Vec3) -> Tuple[float, float]:
    """Decompose the deviation into φ (right +) and ψ (up +), in degrees."""

    p = v_normalize(plan)
    u = v_normalize(axis)
    cos_total = max(-1.0, min(1.0, v_dot(u, p)))
    lateral = v_sub(u, v_scale(p, cos_total))
    right, up = plan_frame(p)
    phi = math.degrees(math.atan2(v_dot(lateral, right), cos_total))
    psi = math.degrees(math.atan2(v_dot(lateral, up), cos_total))
    return phi, psi


def direction_word(phi: float, psi: float, deadband: float = 0.3) -> str:
    """Human direction: 上 / 右上 / 右 / … / 左上."""

    if math.hypot(phi, psi) < deadband:
        return "对准"
    angle = math.degrees(math.atan2(phi, psi)) % 360.0  # 0 = up, 90 = right
    sectors = ("上", "右上", "右", "右下", "下", "左下", "左", "左上")
    return sectors[int((angle + 22.5) // 45.0) % 8]


def tilt_components(axis: Vec3) -> Tuple[float, float, float]:
    """Total tilt from straight down plus its X/Y decomposition (degrees).

    ``tilt_x``/``tilt_y`` stay well behaved even when the needle is almost
    vertical, where an azimuth angle would be undefined.
    """

    a = v_normalize(axis)
    total = v_angle_deg(a, DOWN)
    tilt_x = math.degrees(math.atan2(a[0], -a[2]))
    tilt_y = math.degrees(math.atan2(a[1], -a[2]))
    return total, tilt_x, tilt_y


def display_matrix_from_reference(
    q_ref: Quaternion, needle_axis_sensor: Vec3
) -> Mat3:
    """Backwards-compatible wrapper around :func:`wt901.mapping.display_matrix`."""

    return display_matrix(q_ref, needle_axis_sensor)


@dataclass(frozen=True)
class NeedlePose:
    t_mono: float
    axis: Vec3            # display frame, unit vector, straight down at reference
    quat: Quaternion      # rotation since reference, in the display frame
    tilt_deg: float       # angle from straight down
    tilt_x_deg: float     # left/right
    tilt_y_deg: float     # forward/back
    deviation_deg: float  # angle to the planned direction
    plan: Vec3            # planned direction in the display frame
    deviation_x_deg: float = 0.0  # phi: right +, left -
    deviation_y_deg: float = 0.0  # psi: up +, down -
    direction: str = ""           # 右上 / 左下 / 对准 ...
    valid: bool = True
    # Full model -> display rotation (includes the roll about the needle).
    # None only if the tracker was built without a model placement.
    model_rotation: Optional[Quaternion] = None
    # Device quaternion as received (sensor frame), for logging/diagnostics.
    sensor_quat: Optional[Quaternion] = None

    def as_dict(self) -> dict:
        data = {
            "t": round(self.t_mono, 4),
            "axis": [round(v, 4) for v in self.axis],
            "quat": [round(self.quat.w, 5), round(self.quat.x, 5),
                     round(self.quat.y, 5), round(self.quat.z, 5)],
            "tilt_deg": round(self.tilt_deg, 2),
            "tilt_x_deg": round(self.tilt_x_deg, 2),
            "tilt_y_deg": round(self.tilt_y_deg, 2),
            "deviation_deg": round(self.deviation_deg, 2),
            "deviation_x_deg": round(self.deviation_x_deg, 2),
            "deviation_y_deg": round(self.deviation_y_deg, 2),
            "direction": self.direction,
            "valid": self.valid,
        }
        if self.model_rotation is not None:
            q = self.model_rotation
            data["model_quat"] = [
                round(q.w, 5), round(q.x, 5), round(q.y, 5), round(q.z, 5)
            ]
        if self.sensor_quat is not None:
            q = self.sensor_quat
            data["sensor_quat"] = [
                round(q.w, 5), round(q.x, 5), round(q.y, 5), round(q.z, 5)
            ]
        return data


class NeedleTracker:
    """Converts readings into needle angles.  One entry point: :meth:`update`."""

    def __init__(
        self,
        needle_axis_sensor: Vec3 = (0.0, 0.0, 1.0),
        *,
        smoothing_s: float = 0.06,
        invert_euler: bool = False,
        model_axis: Vec3 = (0.0, 0.0, -1.0),
        model_right: Vec3 = (1.0, 0.0, 0.0),
        invert_quaternion: bool = False,
    ) -> None:
        self._s = v_normalize(needle_axis_sensor)
        self._smoothing = max(0.0, smoothing_s)
        self._invert_euler = invert_euler
        self._invert_quaternion = invert_quaternion
        self._model_axis = v_normalize(model_axis)
        self._model_right = model_right
        self._placement = canonical_placement(self._model_axis, model_right)

        self._q_ref: Optional[Quaternion] = None
        self._plan: Vec3 = DOWN
        self._q_prev: Optional[Quaternion] = None
        self._last_t: Optional[float] = None
        self._u_prev: Optional[Vec3] = None

    # ------------------------------------------------------------- settings
    @property
    def needle_axis(self) -> Vec3:
        return self._s

    @property
    def model_axis(self) -> Vec3:
        return self._model_axis

    @property
    def model_placement(self) -> Quaternion:
        return self._placement

    @property
    def has_reference(self) -> bool:
        return self._q_ref is not None

    @property
    def plan(self) -> Vec3:
        return self._plan

    def set_needle_axis(self, axis: Vec3) -> None:
        """Apply a mounting-calibration result (needle axis in sensor frame)."""

        self._s = v_normalize(axis)
        self._drop_reference()

    def set_model(self, axis: Vec3, right: Vec3 = _X) -> None:
        """Set the model-space needle axis and handle-right direction."""

        self._model_axis = v_normalize(axis)
        self._model_right = right
        self._placement = canonical_placement(self._model_axis, right)
        # The display frame does not depend on the model, so an existing
        # reference stays valid; the next update returns the new placement.

    def capture_reference(self, quat_sensor_to_world: Quaternion) -> None:
        """Freeze the current pose as 0 deg; the plan resets to straight down."""

        self._q_ref = quat_sensor_to_world.normalize()
        self._plan = DOWN
        self._q_prev = self._q_ref
        self._last_t = None
        self._u_prev = None

    def set_plan(self, axis_display: Vec3) -> None:
        self._plan = v_normalize(axis_display)

    def plan_from_current(self) -> None:
        if self._u_prev is not None:
            self._plan = v_normalize(self._u_prev)

    def reset(self) -> None:
        self._drop_reference()

    def _drop_reference(self) -> None:
        self._q_ref = None
        self._plan = DOWN
        self._q_prev = None
        self._last_t = None
        self._u_prev = None

    # ---------------------------------------------------------------- update
    def update(self, reading, now: Optional[float] = None) -> NeedlePose:
        """Consume one SDK reading (quaternion preferred, Euler fallback)."""

        if now is None:
            now = time.monotonic()

        q_now = self._reading_to_quaternion(reading)
        if self._q_ref is None:
            self.capture_reference(q_now)

        q_smoothed = self._smooth(q_now, now)
        assert self._q_ref is not None

        r_display, _ = display_relative_rotation(
            q_smoothed, self._q_ref, self._s
        )
        u = v_normalize(r_display.rotate(DOWN))
        quat = r_display
        model_rotation = quat * self._placement

        self._u_prev = u
        tilt, tilt_x, tilt_y = tilt_components(u)

        deviation = v_angle_deg(u, self._plan)
        phi, psi = plan_deviation(u, self._plan)

        return NeedlePose(
            t_mono=now,
            axis=u,
            quat=quat,
            tilt_deg=tilt,
            tilt_x_deg=tilt_x,
            tilt_y_deg=tilt_y,
            deviation_deg=deviation,
            plan=self._plan,
            deviation_x_deg=phi,
            deviation_y_deg=psi,
            direction=direction_word(phi, psi),
            valid=True,
            model_rotation=model_rotation,
            sensor_quat=q_now,
        )

    # --------------------------------------------------------------- internal
    def _smooth(self, q_now: Quaternion, now: float) -> Quaternion:
        """Rate-independent exponential smoothing on the quaternion itself.

        Slerp keeps the rotation short-path and, unlike smoothing only the
        direction vector, preserves the roll about the needle.
        """

        if (
            self._smoothing <= 0.0
            or self._q_prev is None
            or self._last_t is None
        ):
            self._q_prev = q_now
            self._last_t = now
            return q_now

        dt = max(0.0, min(0.2, now - self._last_t))
        alpha = 1.0 - math.exp(-dt / self._smoothing)
        q_smoothed = self._q_prev.slerp(q_now, alpha)
        self._q_prev = q_smoothed
        self._last_t = now
        return q_smoothed

    def _reading_to_quaternion(self, reading) -> Quaternion:
        try:
            q = quaternion_from_reading(reading)
        except ValueError:
            # A corrupt all-zero quaternion must not kill the acquisition
            # callback; fall back to the Euler angles carried by the same frame.
            q = None
        if q is not None:
            return q.inverse() if self._invert_quaternion else q

        roll = float(getattr(reading, "roll", 0.0))
        pitch = float(getattr(reading, "pitch", 0.0))
        yaw = float(getattr(reading, "yaw", 0.0))
        if self._invert_euler:
            roll, pitch, yaw = -roll, -pitch, -yaw
        return Quaternion.from_euler(roll, pitch, yaw).normalize()

    def calibration_from_vertical(self, quat_sensor_to_world: Quaternion) -> Vec3:
        """Derive ``s`` from one pose where the needle points straight down.

        ``s = R^T * (0,0,-1)``.  Heading is irrelevant because the vertical
        vector is invariant under rotation about the vertical axis, so this
        works without a magnetometer.
        """

        self._s = _calibrate_vertical(quat_sensor_to_world)
        self._drop_reference()
        return self._s

    def calibration_from_two_poses(
        self, poses: Sequence[Tuple[Mat3, Vec3]]
    ) -> Tuple[Vec3, float]:
        """Two-pose axis calibration; returns ``(axis, worst_deviation_deg)``."""

        axis, worst = _calibrate_two_pose(poses)
        self._s = axis
        self._drop_reference()
        return axis, worst
