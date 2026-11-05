"""Needle pose geometry: frames, quaternions, tip kinematics.

Deliberately dependency free (stdlib only), like the rest of the acquisition
core, so it can be unit tested anywhere and reused on a small industrial PC.

Conventions
-----------
* World frame: **Z up**, X/Y horizontal, right handed.  Gravity is ``(0, 0, -1)``.
* Needle frame: the needle points along its local **+Z**.
* ``R`` is a 3x3 rotation matrix mapping *needle frame -> world frame*,
  stored row major as a tuple of three rows.
* A pose that matters for a cylindrical needle is a *direction* plus a point:
  rotation about the needle's own axis is unobservable and irrelevant.

The functions here are the only place where this convention is encoded; the
GUI and the estimator both go through them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

IDENTITY: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
NEEDLE_AXIS_LOCAL: Vec3 = (0.0, 0.0, 1.0)
UP: Vec3 = (0.0, 0.0, 1.0)


# ----------------------------------------------------------------- vectors
def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v_norm(a: Vec3) -> float:
    return math.sqrt(v_dot(a, a))


def v_normalize(a: Vec3) -> Vec3:
    n = v_norm(a)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero vector")
    return (a[0] / n, a[1] / n, a[2] / n)


def v_angle_deg(a: Vec3, b: Vec3) -> float:
    """Angle between two vectors in degrees, robust for near-parallel input."""

    na, nb = v_norm(a), v_norm(b)
    if na < 1e-12 or nb < 1e-12:
        raise ValueError("angle of a zero vector is undefined")
    c = v_dot(a, b) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ----------------------------------------------------------------- matrices
def m_vec(m: Mat3, v: Vec3) -> Vec3:
    return (v_dot(m[0], v), v_dot(m[1], v), v_dot(m[2], v))


def m_mul(a: Mat3, b: Mat3) -> Mat3:
    bt = m_transpose(b)
    return tuple(tuple(v_dot(row, col) for col in bt) for row in a)  # type: ignore[return-value]


def m_transpose(m: Mat3) -> Mat3:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def m_from_columns(c0: Vec3, c1: Vec3, c2: Vec3) -> Mat3:
    return (
        (c0[0], c1[0], c2[0]),
        (c0[1], c1[1], c2[1]),
        (c0[2], c1[2], c2[2]),
    )


def m_to_quaternion(m: Mat3) -> "Quaternion":
    return Quaternion.from_matrix(m)


# -------------------------------------------------------------- quaternions
@dataclass(frozen=True)
class Quaternion:
    """Hamilton quaternion, ``q = w + xi + yj + zk``.

    ``q`` rotates a vector from the body frame to the world frame, i.e.
    ``q.rotate(v)`` implements ``R(q) @ v``.
    """

    w: float
    x: float
    y: float
    z: float

    # ---------------------------------------------------------- constructors
    @classmethod
    def identity(cls) -> "Quaternion":
        return cls(1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_axis_angle(cls, axis: Vec3, angle_deg: float) -> "Quaternion":
        ax = v_normalize(axis)
        half = math.radians(angle_deg) * 0.5
        s = math.sin(half)
        return cls(math.cos(half), ax[0] * s, ax[1] * s, ax[2] * s)

    @classmethod
    def from_euler(cls, roll_deg: float, pitch_deg: float, yaw_deg: float) -> "Quaternion":
        """Intrinsic Z-Y-X (yaw, then pitch, then roll): ``R = Rz Ry Rx``."""

        cr, sr = _cos_sin(roll_deg * 0.5)
        cp, sp = _cos_sin(pitch_deg * 0.5)
        cy, sy = _cos_sin(yaw_deg * 0.5)
        return cls(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    @classmethod
    def from_matrix(cls, m: Mat3) -> "Quaternion":
        trace = m[0][0] + m[1][1] + m[2][2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return cls(0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                       (m[1][0] - m[0][1]) / s)
        if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            return cls((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s,
                       (m[0][2] + m[2][0]) / s)
        if m[1][1] > m[2][2]:
            s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            return cls((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s,
                       (m[1][2] + m[2][1]) / s)
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        return cls((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
                   (m[1][2] + m[2][1]) / s, 0.25 * s)

    # ---------------------------------------------------------------- algebra
    def __mul__(self, other: "Quaternion") -> "Quaternion":
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def conjugate(self) -> "Quaternion":
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def norm(self) -> float:
        return math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self) -> "Quaternion":
        n = self.norm()
        if n < 1e-12:
            raise ValueError("cannot normalise a zero quaternion")
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def inverse(self) -> "Quaternion":
        n = self.norm()
        if n < 1e-12:
            raise ValueError("cannot invert a zero quaternion")
        c = self.conjugate()
        return Quaternion(c.w / (n * n), c.x / (n * n), c.y / (n * n), c.z / (n * n))

    def __neg__(self) -> "Quaternion":
        return Quaternion(-self.w, -self.x, -self.y, -self.z)

    def dot(self, other: "Quaternion") -> float:
        return self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z

    def slerp(self, other: "Quaternion", t: float) -> "Quaternion":
        """Spherical interpolation, shortest path (handles q vs -q)."""

        t = max(0.0, min(1.0, t))
        d = self.dot(other)
        if d < 0.0:
            other = -other
            d = -d
        if d > 0.9995:
            return Quaternion(
                self.w + t * (other.w - self.w),
                self.x + t * (other.x - self.x),
                self.y + t * (other.y - self.y),
                self.z + t * (other.z - self.z),
            ).normalize()
        theta = math.acos(max(-1.0, min(1.0, d)))
        sin_theta = math.sin(theta)
        if abs(sin_theta) < 1e-9:
            return self
        a = math.sin((1.0 - t) * theta) / sin_theta
        b = math.sin(t * theta) / sin_theta
        return Quaternion(
            a * self.w + b * other.w,
            a * self.x + b * other.x,
            a * self.y + b * other.y,
            a * self.z + b * other.z,
        ).normalize()

    @classmethod
    def from_two_vectors(cls, a: Vec3, b: Vec3) -> "Quaternion":
        """Shortest rotation taking unit vector ``a`` onto unit vector ``b``."""

        u = v_normalize(a)
        v = v_normalize(b)
        d = v_dot(u, v)
        if d > 0.999999:
            return cls.identity()
        if d < -0.999999:
            # 180 deg: any axis orthogonal to u works; pick the least aligned basis axis.
            helper = (1.0, 0.0, 0.0) if abs(u[0]) < 0.9 else (0.0, 1.0, 0.0)
            axis = v_cross(u, helper)
            return cls.from_axis_angle(axis, 180.0)
        axis = v_cross(u, v)
        angle = math.degrees(math.acos(max(-1.0, min(1.0, d))))
        return cls.from_axis_angle(axis, angle)


    # ------------------------------------------------------------- conversion
    def to_matrix(self) -> Mat3:
        q = self.normalize()
        w, x, y, z = q.w, q.x, q.y, q.z
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        )

    def to_euler(self) -> Tuple[float, float, float]:
        """``(roll, pitch, yaw)`` in degrees, Z-Y-X convention.

        Yaw is only meaningful with a magnetometer; for a needle use the
        direction vector instead of Euler angles.
        """

        m = self.to_matrix()
        sin_pitch = max(-1.0, min(1.0, -m[2][0]))
        pitch = math.degrees(math.asin(sin_pitch))
        if abs(m[2][0]) < 0.999999:
            roll = math.degrees(math.atan2(m[2][1], m[2][2]))
            yaw = math.degrees(math.atan2(m[1][0], m[0][0]))
        else:  # gimbal lock: fold yaw into roll
            roll = math.degrees(math.atan2(-m[1][2], m[1][1]))
            yaw = 0.0
        return roll, pitch, yaw

    def rotate(self, v: Vec3) -> Vec3:
        return m_vec(self.to_matrix(), v)


def _cos_sin(half_deg: float) -> Tuple[float, float]:
    r = math.radians(half_deg)
    return math.cos(r), math.sin(r)


# --------------------------------------------------------------- needle pose
def needle_axis(r: Mat3) -> Vec3:
    """Needle direction in the world frame (unit vector)."""

    return m_vec(r, NEEDLE_AXIS_LOCAL)


def tip_position(entry: Vec3, r: Mat3, depth_mm: float) -> Vec3:
    """Kinematic tip position: ``p_tip = p_entry + depth * axis``.

    This is the only way an IMU contributes to *position*: orientation plus a
    depth measurement and a known entry point.  Integrating acceleration is not
    an option (see docs/04).
    """

    return v_add(entry, v_scale(needle_axis(r), depth_mm))


def elevation_azimuth_deg(axis: Vec3) -> Tuple[float, float]:
    """Elevation above the horizontal plane and azimuth from +X, in degrees."""

    a = v_normalize(axis)
    elevation = math.degrees(math.asin(max(-1.0, min(1.0, a[2]))))
    azimuth = math.degrees(math.atan2(a[1], a[0]))
    return elevation, azimuth


def deviation_deg(current: Vec3, planned: Vec3) -> float:
    """Angular error between the actual and the planned direction."""

    return v_angle_deg(current, planned)


def predicted_tip_error_mm(depth_mm: float, angle_error_deg: float) -> float:
    """Lateral tip error caused by an angular error at a given depth."""

    return abs(depth_mm) * math.sin(math.radians(angle_error_deg))


def cone_angle_deg(axis: Vec3, cone_axis: Vec3 = UP) -> float:
    """Angle between the needle and a cone axis (default: vertical)."""

    return v_angle_deg(axis, cone_axis)


# ------------------------------------------------------------------ alignment
def frame_from_two_vectors(primary: Vec3, secondary: Vec3) -> Mat3:
    """Orthonormal frame whose X axis is ``primary`` and Z axis is ``primary x secondary``.

    Used for the mounting calibration: two non-parallel needle directions in the
    sensor frame and the same two directions in the needle/world frame are
    enough to solve the rigid rotation between them.
    """

    e1 = v_normalize(primary)
    proj = v_scale(e1, v_dot(secondary, e1))
    e2 = v_normalize(v_sub(secondary, proj))
    e3 = v_cross(e1, e2)
    return m_from_columns(e1, e2, e3)


def align_frames(
    sensor_primary: Vec3,
    sensor_secondary: Vec3,
    world_primary: Vec3,
    world_secondary: Vec3,
) -> Mat3:
    """Solve ``R`` with ``R @ sensor_vector = world_vector`` from two poses.

    Two non-parallel directions are enough: build a frame in each space and
    ``R = W @ S^T``.  This is the two-pose mounting calibration for the sensor
    on the needle (no SVD needed, no numerical surprises).

    Raises :class:`ValueError` if either pair is (nearly) parallel.
    """

    if v_angle_deg(sensor_primary, sensor_secondary) < 5.0:
        raise ValueError("the two sensor poses are nearly parallel; use a larger angle")
    if v_angle_deg(world_primary, world_secondary) < 5.0:
        raise ValueError("the two world poses are nearly parallel; use a larger angle")
    s = frame_from_two_vectors(sensor_primary, sensor_secondary)
    w = frame_from_two_vectors(world_primary, world_secondary)
    return m_mul(w, m_transpose(s))


def rotation_difference_deg(a: Mat3, b: Mat3) -> float:
    """Angle of ``a^T b``, i.e. how far two orientations differ."""

    rel = m_mul(m_transpose(a), b)
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5))))


# --------------------------------------------------- mounting calibration (s)
# Only the needle *axis* is needed: a cylindrical needle has no meaningful roll,
# so the mounting calibration reduces to one unit vector ``s`` -- the needle
# direction expressed in the sensor frame.  Everything else is redundant.
def needle_axis_in_sensor(poses: Sequence[Tuple[Mat3, Vec3]]) -> Tuple[Vec3, float]:
    """Estimate the needle axis in the sensor frame from static poses.

    ``poses`` is a list of ``(R_sensor_to_world, known_world_direction)`` pairs,
    e.g. the needle held vertically and then horizontally in a jig.  For each
    pose the axis in the sensor frame is ``R^T w``; the estimate is their mean
    direction and the return also carries the worst deviation in degrees
    (a mounting-slip detector: keep it below ~1 deg).

    Raises :class:`ValueError` when fewer than two poses are given or the
    observations are mutually inconsistent (opposite directions).
    """

    if len(poses) < 2:
        raise ValueError("need at least two poses to calibrate")

    vectors: List[Vec3] = []
    for r, w in poses:
        v = m_vec(m_transpose(r), w)
        vectors.append(v_normalize(v))

    total = (0.0, 0.0, 0.0)
    for v in vectors:
        total = v_add(total, v)
    if v_norm(total) < 1e-6:
        raise ValueError("poses are inconsistent (axis flips), check the world directions")

    s = v_normalize(total)
    worst = max(v_angle_deg(v, s) for v in vectors)
    return s, worst


def axis_world(r_sensor_to_world: Mat3, needle_axis_sensor: Vec3) -> Vec3:
    """Needle direction in the world frame: ``u = R @ s``."""

    return v_normalize(m_vec(r_sensor_to_world, needle_axis_sensor))


def tip_from_axis(entry: Vec3, axis: Vec3, depth_mm: float) -> Vec3:
    """Tip position from a known entry point, direction and insertion depth."""

    return v_add(entry, v_scale(v_normalize(axis), depth_mm))


def relative_axis(r_ref: Mat3, r_now: Mat3, needle_axis_sensor: Vec3) -> Vec3:
    """Needle direction relative to a reference pose.

    This is what makes the system usable without a magnetometer: the reference
    pose becomes the zero, so heading drift and magnetic disturbance stop
    mattering as long as the procedure stays short.
    """

    rel = m_mul(m_transpose(r_ref), r_now)
    return axis_world(rel, needle_axis_sensor)
