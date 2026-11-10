"""Sensor pose -> imported needle model pose.

This module is the single place where the *mapping* between the IMU and the
needle model is defined.  Everything else (estimator, renderer, calibration)
goes through these functions, so a mounting change is a one-line config change
instead of a hunt through the GUI code.

Frames and conventions
----------------------
``S``  sensor body frame, as used by the device quaternion.
``W``  world frame, Z up (right handed).  The device reports ``q_SW``.
``M``  needle model / CAD assembly frame (the coordinates inside the STL/STEP).
``D``  display frame used by the 3D view, Z up, needle points down at reference.

The device quaternion ``q_SW`` rotates a vector *from S to W*::

    v_W = R(q_SW) @ v_S

The needle axis is a constant ``s`` in the sensor frame (``s = R_MS @ u_M``,
where ``R_MS`` is the mounting rotation).  At run time::

    u_W(t) = R(q_SW(t)) @ s

Relative motion since the reference pose ``q0`` is a pure world rotation::

    R_rel(t) = R(q_SW(t)) @ R(q_SW(0))^T

The mounting rotation cancels out of ``R_rel``, so a rigid needle only needs
the *axis* ``s`` -- not the full mounting matrix.  That is why calibration can
be one static pose (needle vertical) instead of a full hand-eye procedure.

Display frame
-------------
``D`` is frozen at the reference pose: its Z axis is opposite the needle, so
the needle reads straight down at reference.  Its X axis is the sensor X axis
projected onto the display plane; this only fixes the (physically arbitrary)
roll of the virtual camera, not the needle motion.

The rotation shown for the model is therefore::

    R_D_rel(t) = M_disp @ R_rel(t) @ M_disp^T          (in D)
    P_ref      = canonical placement M -> D at reference
    R_model(t) = R_D_rel(t) @ P_ref

``P_ref`` only fixes how the model sits at the reference instant (its needle
axis is mapped to straight down, and ``model_right`` to display +X).  Every
subsequent rotation is applied to that placement, including rotations *about*
the needle axis.  The former implementation used the shortest arc from the
model axis to the current axis; that loses the roll and makes a handle/PCB
model spin inconsistently when the needle is rolled.  Do not go back to it.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from .pose import (
    Mat3,
    Quaternion,
    Vec3,
    frame_from_two_vectors,
    m_from_columns,
    m_mul,
    m_transpose,
    m_vec,
    needle_axis_in_sensor,
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


# --------------------------------------------------------------------- helpers
def _normalised(q: Quaternion) -> Quaternion:
    try:
        return q.normalize()
    except ValueError as exc:
        raise ValueError("zero quaternion: the device sent an invalid pose") from exc


def _orthogonal_hint(axis: Vec3, hint: Vec3) -> Vec3:
    """Project ``hint`` onto the plane perpendicular to ``axis``, robustly."""

    a = v_normalize(axis)
    projected = v_sub(hint, v_scale(a, v_dot(hint, a)))
    if v_norm(projected) > 1e-6:
        return v_normalize(projected)
    for fallback in (_X, _Y, _Z):
        projected = v_sub(fallback, v_scale(a, v_dot(fallback, a)))
        if v_norm(projected) > 1e-6:
            return v_normalize(projected)
    raise ValueError("cannot build a frame perpendicular to a degenerate axis")


# ------------------------------------------------------------------ display
def display_matrix(
    q_ref: Quaternion,
    needle_axis_sensor: Vec3,
    *,
    right_hint_world: Optional[Vec3] = None,
) -> Mat3:
    """World -> display rotation frozen at the reference pose.

    Display +Z is the *opposite* of the reference needle direction, so the
    needle points down (``(0, 0, -1)``) at reference.  ``right_hint_world``
    (default: the sensor +X image) only chooses the display roll; the needle
    motion itself does not depend on it.
    """

    r_ref = _normalised(q_ref).to_matrix()
    u_ref = v_normalize(m_vec(r_ref, needle_axis_sensor))
    z = v_scale(u_ref, -1.0)

    hint = m_vec(r_ref, _X) if right_hint_world is None else right_hint_world
    x = _orthogonal_hint(z, hint)
    y = v_cross(z, x)

    # Rows are the display basis expressed in world coordinates, i.e. the
    # transpose of the column matrix.
    return m_transpose(m_from_columns(x, y, z))


def reference_axis(
    q_ref: Quaternion, needle_axis_sensor: Vec3
) -> Vec3:
    """Needle direction in the world at the reference pose."""

    return v_normalize(m_vec(_normalised(q_ref).to_matrix(), needle_axis_sensor))


def axis_in_sensor(
    q_sensor_to_world: Quaternion, needle_axis_sensor: Vec3
) -> Vec3:
    """Needle direction in the world for a device quaternion."""

    return v_normalize(
        m_vec(_normalised(q_sensor_to_world).to_matrix(), needle_axis_sensor)
    )


def axis_in_display(
    q_now: Quaternion,
    q_ref: Quaternion,
    needle_axis_sensor: Vec3,
    *,
    right_hint_world: Optional[Vec3] = None,
) -> Vec3:
    """Needle direction in the display frame (straight down at reference)."""

    m_disp = display_matrix(q_ref, needle_axis_sensor, right_hint_world=right_hint_world)
    u_world = axis_in_sensor(q_now, needle_axis_sensor)
    return v_normalize(m_vec(m_disp, u_world))


# ------------------------------------------------------------- relative pose
def relative_rotation(q_now: Quaternion, q_ref: Quaternion) -> Quaternion:
    """World rotation since the reference: ``R(q_now) @ R(q_ref)^T``."""

    return _normalised(_normalised(q_now) * _normalised(q_ref).conjugate())


def display_relative_rotation(
    q_now: Quaternion,
    q_ref: Quaternion,
    needle_axis_sensor: Vec3,
    *,
    right_hint_world: Optional[Vec3] = None,
) -> Tuple[Quaternion, Mat3]:
    """Rotation since reference, expressed in the display frame.

    Returns ``(R_D_rel, M_disp)``.  ``R_D_rel`` is identity at reference and
    maps the reference needle direction ``DOWN`` to the current direction.
    """

    m_disp = display_matrix(q_ref, needle_axis_sensor, right_hint_world=right_hint_world)
    r_rel = relative_rotation(q_now, q_ref).to_matrix()
    r_display = m_mul(m_mul(m_disp, r_rel), m_transpose(m_disp))
    return _normalised(Quaternion.from_matrix(r_display)), m_disp


# ----------------------------------------------------------- model placement
def canonical_placement(
    model_axis: Vec3 = _Y, model_right: Vec3 = _X
) -> Quaternion:
    """Fixed model -> display placement at the reference pose.

    The model's needle axis is mapped to straight down and ``model_right`` to
    display +X.  For a mesh whose needle already points down -Z this is the
    identity; for the CAD assembly (needle +Y) it is the -90 deg X rotation.
    """

    axis = v_normalize(model_axis)
    right = _orthogonal_hint(axis, model_right)

    model_frame = frame_from_two_vectors(right, axis)       # X=right, Z=right x axis
    display_frame = frame_from_two_vectors(_X, DOWN)        # X=display X, Z=display Y
    placement = m_mul(display_frame, m_transpose(model_frame))
    return _normalised(Quaternion.from_matrix(placement))


def model_rotation(
    q_now: Quaternion,
    q_ref: Quaternion,
    needle_axis_sensor: Vec3,
    model_axis: Vec3 = _Y,
    model_right: Vec3 = _X,
    *,
    right_hint_world: Optional[Vec3] = None,
) -> Quaternion:
    """Full model -> display rotation at ``q_now``.

    Unlike a shortest-arc axis alignment this keeps the roll about the needle,
    so a handle/PCB model follows the real assembly exactly.
    """

    r_display, _ = display_relative_rotation(
        q_now, q_ref, needle_axis_sensor, right_hint_world=right_hint_world
    )
    return _normalised(r_display * canonical_placement(model_axis, model_right))


def model_axis_in_display(
    q_now: Quaternion,
    q_ref: Quaternion,
    needle_axis_sensor: Vec3,
    model_axis: Vec3 = _Y,
    model_right: Vec3 = _X,
) -> Vec3:
    """Convenience: where the model's needle axis points in the display frame."""

    return v_normalize(
        model_rotation(
            q_now, q_ref, needle_axis_sensor, model_axis, model_right
        ).rotate(model_axis)
    )


# --------------------------------------------------------------- calibration
def calibrate_axis_vertical(
    q_vertical: Quaternion, world_down: Vec3 = DOWN
) -> Vec3:
    """Needle axis in the sensor frame from one vertical pose.

    ``s = R(q)^T @ (0, 0, -1)``.  Only gravity is used, so no magnetometer and
    no knowledge of heading is required.  The returned vector points from the
    hub towards the tip.
    """

    r = _normalised(q_vertical).to_matrix()
    return v_normalize(m_vec(m_transpose(r), world_down))


def calibrate_axis_from_poses(
    poses: Sequence[Tuple[Mat3, Vec3]]
) -> Tuple[Vec3, float]:
    """Two-pose calibration; returns ``(axis, worst_deviation_deg)``.

    ``poses`` is a sequence of ``(R_sensor_to_world, known_world_direction)``
    pairs (e.g. vertical, then horizontal against a square).  Delegates to
    :func:`wt901.pose.needle_axis_in_sensor`; keep the residual below ~1 deg.
    """

    return needle_axis_in_sensor(poses)


def axis_error_deg(quat: Quaternion, model_axis: Vec3, expected: Vec3) -> float:
    """Angle between the model axis after ``quat`` and an expected direction."""

    actual = v_normalize(quat.rotate(model_axis))
    dot = max(-1.0, min(1.0, v_dot(actual, v_normalize(expected))))
    return math.degrees(math.acos(dot))


# --------------------------------------------------------- quaternion sources
def quaternion_from_reading(reading) -> Optional[Quaternion]:
    """Extract a device quaternion from a reading, if it carries one.

    Supports ``reading.quaternion`` (a ``Quaternion`` or a 4-tuple in
    ``(w, x, y, z)`` order) and the flat ``qw/qx/qy/qz`` fields used by
    :class:`wt901.model.ImuSample`.  Returns ``None`` when the reading has no
    quaternion, in which case the caller falls back to Euler angles.
    """

    q = getattr(reading, "quaternion", None)
    if callable(q):
        q = q()
    if q is not None:
        if isinstance(q, Quaternion):
            return _normalised(q)
        try:
            w, x, y, z = (float(v) for v in q)  # type: ignore[misc]
        except (TypeError, ValueError) as exc:
            raise ValueError("reading.quaternion must be a Quaternion or (w,x,y,z)") from exc
        return _normalised(Quaternion(w, x, y, z))

    fields = tuple(getattr(reading, name, None) for name in ("qw", "qx", "qy", "qz"))
    if all(value is not None for value in fields):
        w, x, y, z = (float(value) for value in fields)  # type: ignore[arg-type]
        return _normalised(Quaternion(w, x, y, z))
    return None


def reading_quaternion(reading, *, invert: bool = False) -> Optional[Quaternion]:
    """Like :func:`quaternion_from_reading`, optionally inverting the convention.

    Set ``invert=True`` when the device reports world -> sensor instead of
    sensor -> world.  On real hardware this is obvious: tilting the needle to
    the operator's right makes the virtual needle go left.
    """

    q = quaternion_from_reading(reading)
    if q is None:
        return None
    return q.inverse() if invert else q
