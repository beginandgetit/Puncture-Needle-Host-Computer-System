"""Sensor -> needle-model mapping tests.

These tests are written against an independent ground truth: a mounting
rotation R_MS (model -> sensor), a reference pose, a world-space motion, and
the display frame built from those.  If the mapping drops the roll about the
needle (the old shortest-arc behaviour) these tests fail.
"""

from __future__ import annotations

import math
import unittest

from wt901.mapping import (
    axis_error_deg,
    axis_in_display,
    axis_in_sensor,
    calibrate_axis_from_poses,
    calibrate_axis_vertical,
    canonical_placement,
    display_matrix,
    display_relative_rotation,
    model_rotation,
    quaternion_from_reading,
    reading_quaternion,
    relative_rotation,
)
from wt901.pose import (
    IDENTITY,
    Quaternion,
    m_mul,
    m_transpose,
    m_vec,
    v_angle_deg,
    v_normalize,
)

DOWN = (0.0, 0.0, -1.0)
X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)

MODEL_AXIS = Y
MODEL_RIGHT = X


def _mat_close(a, b, tol: float = 1e-9) -> bool:
    return all(abs(a[i][j] - b[i][j]) <= tol for i in range(3) for j in range(3))


def _vec_close(a, b, tol: float = 1e-9) -> bool:
    return all(abs(a[i] - b[i]) <= tol for i in range(3))


class TestDisplayFrame(unittest.TestCase):
    def test_reference_needle_points_down(self) -> None:
        for q in (
            Quaternion.identity(),
            Quaternion.from_euler(10, 20, 30),
            Quaternion.from_axis_angle((1, 1, 0), 75),
        ):
            m = display_matrix(q, Y)
            u = axis_in_sensor(q, Y)
            self.assertTrue(_vec_close(m_vec(m, u), DOWN, 1e-9), m_vec(m, u))

    def test_display_matrix_is_orthonormal(self) -> None:
        q = Quaternion.from_euler(-25, 40, 115)
        m = display_matrix(q, (0.2, -0.9, 0.3))
        self.assertTrue(_mat_close(m_mul(m, m_transpose(m)), IDENTITY, 1e-9))

    def test_right_hint_only_changes_roll(self) -> None:
        q = Quaternion.identity()
        m_a = display_matrix(q, Y)
        m_b = display_matrix(q, Y, right_hint_world=Z)
        # The vertical (needle) row is identical; only the in-plane X/Y rows
        # differ by a rotation about the display Z axis.
        self.assertTrue(_vec_close(m_a[2], m_b[2], 1e-12))


class TestCanonicalPlacement(unittest.TestCase):
    def test_cad_axis_maps_to_down_and_right_to_x(self) -> None:
        p = canonical_placement(MODEL_AXIS, MODEL_RIGHT)
        self.assertTrue(_vec_close(p.rotate(MODEL_AXIS), DOWN, 1e-9))
        self.assertTrue(_vec_close(p.rotate(MODEL_RIGHT), X, 1e-9))

    def test_mesh_pointing_down_is_identity(self) -> None:
        p = canonical_placement((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
        self.assertTrue(_mat_close(p.to_matrix(), IDENTITY, 1e-9))

    def test_non_orthogonal_right_is_projected(self) -> None:
        p = canonical_placement(Y, (1.0, 1.0, 0.0))
        self.assertTrue(_vec_close(p.rotate(Y), DOWN, 1e-9))
        self.assertAlmostEqual(p.rotate((1.0, 1.0, 0.0))[0], 1.0, places=9)

    def test_parallel_right_falls_back(self) -> None:
        p = canonical_placement(Y, Y)  # degenerate handle-right hint
        self.assertTrue(_vec_close(p.rotate(Y), DOWN, 1e-9))


class TestRelativeRotation(unittest.TestCase):
    def test_reference_is_identity(self) -> None:
        q = Quaternion.from_euler(10, 20, 30)
        self.assertTrue(_mat_close(relative_rotation(q, q).to_matrix(), IDENTITY, 1e-9))
        r_display, _ = display_relative_rotation(q, q, Y)
        self.assertTrue(_mat_close(r_display.to_matrix(), IDENTITY, 1e-9))

    def test_world_rotation_is_conjugated_into_display(self) -> None:
        q_ref = Quaternion.from_euler(10, 20, 30)
        q_rel = Quaternion.from_euler(15, -25, 40)
        q_now = q_rel * q_ref
        m = display_matrix(q_ref, Y)
        expected = m_mul(m_mul(m, q_rel.to_matrix()), m_transpose(m))
        actual, _ = display_relative_rotation(q_now, q_ref, Y)
        self.assertTrue(_mat_close(actual.to_matrix(), expected, 1e-9))

    def test_model_axis_follows_display_axis(self) -> None:
        q_ref = Quaternion.from_euler(-10, 25, 60)
        for q_now in (
            Quaternion.from_euler(5, 6, 7),
            Quaternion.from_axis_angle((1, -2, 3), 80),
            Quaternion.from_axis_angle(Y, 35),  # pure roll about the needle
        ):
            qm = model_rotation(q_now, q_ref, Y, MODEL_AXIS, MODEL_RIGHT)
            self.assertLess(
                v_angle_deg(qm.rotate(MODEL_AXIS), axis_in_display(q_now, q_ref, Y)),
                1e-5,
            )


class TestFullPoseCorrespondence(unittest.TestCase):
    """Ground truth: model -> sensor mounting, reference pose, world motion."""

    def setUp(self) -> None:
        self.mounting = Quaternion.from_euler(20, -35, 50)  # model -> sensor
        self.s = v_normalize(self.mounting.rotate(MODEL_AXIS))
        self.q_ref = Quaternion.from_euler(10, 20, 30)
        self.q_rel = Quaternion.from_euler(15, -25, 40)
        self.q_now = self.q_rel * self.q_ref
        self.r_ref = self.q_ref.to_matrix()
        self.r_mount = self.mounting.to_matrix()
        # Anchor the display X axis to the model's right direction so the
        # reference placement equals the true CAD orientation (no free roll).
        right_world = m_vec(self.r_ref, m_vec(self.r_mount, MODEL_RIGHT))
        self.m = display_matrix(self.q_ref, self.s, right_hint_world=right_world)
        self.p = canonical_placement(MODEL_AXIS, MODEL_RIGHT)

    def test_reference_placement_matches_ground_truth(self) -> None:
        expected = m_mul(m_mul(self.m, self.r_ref), self.r_mount)
        self.assertTrue(_mat_close(self.p.to_matrix(), expected, 1e-9))

    def test_arbitrary_model_point_matches_ground_truth(self) -> None:
        qm = model_rotation(
            self.q_now,
            self.q_ref,
            self.s,
            MODEL_AXIS,
            MODEL_RIGHT,
            right_hint_world=m_vec(self.r_ref, m_vec(self.r_mount, MODEL_RIGHT)),
        )
        for point in ((1.0, 2.0, 3.0), (-5.0, 0.5, 9.0), MODEL_RIGHT, MODEL_AXIS):
            at_reference = m_vec(self.r_ref, m_vec(self.r_mount, point))
            at_now = m_vec(self.q_rel.to_matrix(), at_reference)
            expected = m_vec(self.m, at_now)
            self.assertTrue(_vec_close(qm.rotate(point), expected, 1e-9), point)


class TestRoll(unittest.TestCase):
    def test_roll_about_needle_rotates_handle(self) -> None:
        q_ref = Quaternion.identity()
        placement = canonical_placement(MODEL_AXIS, MODEL_RIGHT)
        right_ref = placement.rotate(MODEL_RIGHT)

        rolled = model_rotation(
            Quaternion.from_axis_angle(Y, 30.0), q_ref, Y, MODEL_AXIS, MODEL_RIGHT
        )
        # The needle axis does not move ...
        self.assertLess(v_angle_deg(rolled.rotate(MODEL_AXIS), DOWN), 1e-7)
        # ... but the handle-right vector does, by exactly the roll angle.
        self.assertAlmostEqual(
            v_angle_deg(rolled.rotate(MODEL_RIGHT), right_ref), 30.0, places=6
        )

    def test_shortest_arc_loses_roll_but_mapping_keeps_it(self) -> None:
        q_ref = Quaternion.identity()
        axis = axis_in_display(Quaternion.from_axis_angle(Y, 30.0), q_ref, Y)
        shortest = Quaternion.from_two_vectors(MODEL_AXIS, axis)
        placement = canonical_placement(MODEL_AXIS, MODEL_RIGHT)
        right_ref = placement.rotate(MODEL_RIGHT)
        right_shortest = shortest.rotate(MODEL_RIGHT)
        self.assertAlmostEqual(
            v_angle_deg(right_shortest, right_ref), 0.0, places=6
        )  # regression: the old path cannot represent roll

        full = model_rotation(
            Quaternion.from_axis_angle(Y, 30.0), q_ref, Y, MODEL_AXIS, MODEL_RIGHT
        )
        self.assertAlmostEqual(
            v_angle_deg(full.rotate(MODEL_RIGHT), right_ref), 30.0, places=6
        )


class TestCalibration(unittest.TestCase):
    def test_vertical_calibration_recovers_axis(self) -> None:
        for axis in ((0.0, 1.0, 0.0), (0.2, -0.9, 0.3), (0.0, 0.0, -1.0)):
            q = Quaternion.from_two_vectors(axis, DOWN)
            recovered = calibrate_axis_vertical(q)
            self.assertLess(v_angle_deg(recovered, axis), 1e-6, axis)

    def test_two_pose_calibration_recovers_axis(self) -> None:
        axis = v_normalize((0.1, 0.9, -0.4))
        q1 = Quaternion.from_two_vectors(axis, DOWN)
        q2 = Quaternion.from_two_vectors(axis, X)
        recovered, worst = calibrate_axis_from_poses(
            [(q1.to_matrix(), DOWN), (q2.to_matrix(), X)]
        )
        self.assertLess(v_angle_deg(recovered, axis), 1e-6)
        self.assertLess(worst, 0.01)

    def test_axis_error_helper(self) -> None:
        q = Quaternion.from_two_vectors(Y, X)
        self.assertAlmostEqual(axis_error_deg(q, Y, X), 0.0, places=6)


class _Reading:
    def __init__(self, quaternion=None, **fields):
        if quaternion is not None:
            self.quaternion = quaternion
        for name, value in fields.items():
            setattr(self, name, value)


class TestQuaternionSources(unittest.TestCase):
    def test_tuple_quaternion(self) -> None:
        q = Quaternion.from_euler(10, 20, 30)
        r = _Reading(quaternion=(q.w, q.x, q.y, q.z))
        self.assertGreater(abs(quaternion_from_reading(r).dot(q)), 0.9999999)

    def test_flat_fields(self) -> None:
        q = Quaternion.from_euler(-5, 15, 45)
        r = _Reading(qw=q.w, qx=q.x, qy=q.y, qz=q.z)
        self.assertGreater(abs(quaternion_from_reading(r).dot(q)), 0.9999999)

    def test_none_when_absent(self) -> None:
        self.assertIsNone(quaternion_from_reading(_Reading(roll=0.0)))

    def test_invert_convention(self) -> None:
        q = Quaternion.from_euler(10, 20, 30)
        r = _Reading(quaternion=(q.w, q.x, q.y, q.z))
        inverted = reading_quaternion(r, invert=True)
        self.assertGreater(abs(inverted.dot(q.inverse())), 0.9999999)

    def test_zero_quaternion_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            quaternion_from_reading(_Reading(quaternion=(0.0, 0.0, 0.0, 0.0)))


class TestAxisHelpers(unittest.TestCase):
    def test_axis_in_sensor_is_normalised(self) -> None:
        q = Quaternion.from_euler(30, 40, 50)
        self.assertAlmostEqual(
            math.sqrt(sum(c * c for c in axis_in_sensor(q, (0.0, 2.0, 0.0)))), 1.0
        )


if __name__ == "__main__":
    unittest.main()
