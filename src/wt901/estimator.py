"""Needle angle tracking: IMU orientation -> needle direction -> UI angles.

Scope (confirmed): **angle guidance only**.  No position, no insertion depth,
no external tracking.

Definitions
-----------
``s``   needle axis expressed in the *sensor* frame (constant after mounting;
        default assumes the sensor +Z is parallel to the needle).
``R``   sensor -> world rotation built from the device Euler angles.
``R0``  reference pose, captured by the operator.  Every displayed angle is
        relative to it, so magnetic north is never needed: the hospital room is
        full of steel and the magnetometer cannot be trusted.

Display frame
-------------
Built from the reference pose itself: its Z axis points *up* (opposite the
needle at reference) and its X axis keeps the sensor's own X sense.  Therefore

* at the moment of capture, the needle reads exactly "straight down" (0, 0, -1);
* a roll about the sensor X axis shows up as a forward/back tilt, not a
  sideways one, which matches what the operator sees.

Rotating about the vertical is free: for a cylindrical needle heading has no
meaning, and that is precisely why this works without a magnetometer.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from .pose import (
    Mat3,
    Quaternion,
    Vec3,
    m_from_columns,
    m_vec,
    m_transpose,
    v_add,
    v_angle_deg,
    v_cross,
    v_dot,
    v_norm,
    v_normalize,
    v_scale,
    v_sub,
)

DOWN: Vec3 = (0.0, 0.0, -1.0)


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


def display_matrix_from_reference(q_ref: Quaternion, needle_axis_sensor: Vec3) -> Mat3:
    """World -> display rotation that makes the reference pose read straight down."""

    r = q_ref.to_matrix()
    needle_world = v_normalize(m_vec(r, needle_axis_sensor))
    z = v_scale(needle_world, -1.0)                 # display "up" in world coords
    x_sensor = (r[0][0], r[1][0], r[2][0])          # image of sensor +X
    x = v_sub(x_sensor, v_scale(z, v_dot(x_sensor, z)))
    if v_norm(x) < 1e-6:                            # degenerate: pick any perpendicular
        helper = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
        x = v_cross(z, helper)
    x = v_normalize(x)
    y = v_cross(z, x)
    # columns x, y, z = display basis expressed in world; the transpose maps world -> display
    return m_transpose(m_from_columns(x, y, z))


@dataclass(frozen=True)
class NeedlePose:
    t_mono: float
    axis: Vec3            # display frame, unit vector
    quat: Quaternion      # needle orientation in the display frame
    tilt_deg: float       # angle from straight down
    tilt_x_deg: float     # left/right
    tilt_y_deg: float     # forward/back
    deviation_deg: float  # angle to the planned direction
    plan: Vec3            # planned direction in the display frame
    valid: bool = True

    def as_dict(self) -> dict:
        return {
            "t": round(self.t_mono, 4),
            "axis": [round(v, 4) for v in self.axis],
            "quat": [round(self.quat.w, 5), round(self.quat.x, 5),
                     round(self.quat.y, 5), round(self.quat.z, 5)],
            "tilt_deg": round(self.tilt_deg, 2),
            "tilt_x_deg": round(self.tilt_x_deg, 2),
            "tilt_y_deg": round(self.tilt_y_deg, 2),
            "deviation_deg": round(self.deviation_deg, 2),
            "valid": self.valid,
        }


class NeedleTracker:
    """Converts readings into needle angles.  One entry point: :meth:`update`."""

    def __init__(
        self,
        needle_axis_sensor: Vec3 = (0.0, 0.0, 1.0),
        *,
        smoothing_s: float = 0.06,
        invert_euler: bool = False,
    ) -> None:
        self._s = v_normalize(needle_axis_sensor)
        self._smoothing = max(0.0, smoothing_s)
        self._invert = invert_euler

        self._q_ref: Optional[Quaternion] = None
        self._m_disp: Optional[Mat3] = None
        self._plan: Vec3 = DOWN
        self._u_prev: Optional[Vec3] = None
        self._last_t: Optional[float] = None

    # ------------------------------------------------------------- settings
    @property
    def needle_axis(self) -> Vec3:
        return self._s

    @property
    def has_reference(self) -> bool:
        return self._q_ref is not None

    @property
    def plan(self) -> Vec3:
        return self._plan

    def set_needle_axis(self, axis: Vec3) -> None:
        """Apply a mounting-calibration result (needle axis in sensor frame)."""

        self._s = v_normalize(axis)
        self._q_ref = None
        self._m_disp = None
        self._u_prev = None

    def capture_reference(self, quat_sensor_to_world: Quaternion) -> None:
        """Freeze the current pose as 0 deg; the plan resets to straight down."""

        self._q_ref = quat_sensor_to_world.normalize()
        self._m_disp = display_matrix_from_reference(self._q_ref, self._s)
        self._plan = DOWN
        self._u_prev = None

    def set_plan(self, axis_display: Vec3) -> None:
        self._plan = v_normalize(axis_display)

    def plan_from_current(self) -> None:
        if self._u_prev is not None:
            self._plan = v_normalize(self._u_prev)

    def reset(self) -> None:
        self._q_ref = None
        self._m_disp = None
        self._plan = DOWN
        self._u_prev = None

    # ---------------------------------------------------------------- update
    def update(self, reading, now: Optional[float] = None) -> NeedlePose:
        """Consume one SDK reading (``roll``/``pitch``/``yaw`` in degrees)."""

        if now is None:
            now = time.monotonic()

        q_now = self._reading_to_quaternion(reading)
        if self._q_ref is None:
            self.capture_reference(q_now)

        assert self._m_disp is not None
        u_world = v_normalize(m_vec(q_now.to_matrix(), self._s))
        u = v_normalize(m_vec(self._m_disp, u_world))

        if self._smoothing > 0.0 and self._u_prev is not None and self._last_t is not None:
            # Exponential smoothing on the direction vector, weighted by the
            # real sample interval so behaviour is rate independent.
            dt = max(0.0, min(0.2, now - self._last_t))
            alpha = 1.0 - math.exp(-dt / self._smoothing)
            u = v_normalize(v_add(v_scale(self._u_prev, 1.0 - alpha), v_scale(u, alpha)))

        self._u_prev = u
        self._last_t = now
        tilt, tilt_x, tilt_y = tilt_components(u)
        # Canonical needle orientation as a quaternion: the shortest rotation
        # from "straight down".  Twist about the needle axis is deliberately
        # dropped (a cylindrical needle has no meaningful roll).
        quat = Quaternion.from_two_vectors(DOWN, u)

        return NeedlePose(
            t_mono=now,
            axis=u,
            quat=quat,
            tilt_deg=tilt,
            tilt_x_deg=tilt_x,
            tilt_y_deg=tilt_y,
            deviation_deg=v_angle_deg(u, self._plan),
            plan=self._plan,
            valid=True,
        )

    # --------------------------------------------------------------- internal
    def _reading_to_quaternion(self, reading) -> Quaternion:
        roll = float(reading.roll)
        pitch = float(reading.pitch)
        yaw = float(reading.yaw)
        if self._invert:
            roll, pitch, yaw = -roll, -pitch, -yaw
        return Quaternion.from_euler(roll, pitch, yaw)

    def calibration_from_vertical(self, quat_sensor_to_world: Quaternion) -> Vec3:
        """Derive ``s`` from one pose where the needle points straight down.

        ``s = R^T * (0,0,-1)``.  Heading is irrelevant because the vertical
        vector is invariant under rotation about the vertical axis, so this
        works without a magnetometer.
        """

        r = quat_sensor_to_world.to_matrix()
        s = v_normalize(m_vec(m_transpose(r), DOWN))
        self._s = s
        self._q_ref = None
        self._m_disp = None
        return s
