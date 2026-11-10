"""Geometry tests: conventions, kinematics, mounting calibration."""

from __future__ import annotations

import math
import unittest

from wt901.pose import (
    IDENTITY,
    Quaternion,
    align_frames,
    axis_world,
    cone_angle_deg,
    deviation_deg,
    elevation_azimuth_deg,
    frame_from_two_vectors,
    m_mul,
    needle_axis,
    needle_axis_in_sensor,
    predicted_tip_error_mm,
    relative_axis,
    rotate_about,
    rotation_difference_deg,
    tip_from_axis,
    tip_position,
    v_dot,
    v_norm,
)


def close(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) <= tol


class TestQuaternion(unittest.TestCase):
    def test_identity(self) -> None:
        self.assertEqual(Quaternion.identity().to_matrix(), IDENTITY)

    def test_yaw_90(self) -> None:
        m = Quaternion.from_euler(0, 0, 90).to_matrix()
        x = (m[0][0], m[1][0], m[2][0])  # image of (1,0,0)
        self.assertAlmostEqual(x[0], 0.0, places=9)
        self.assertAlmostEqual(x[1], 1.0, places=9)

    def test_composition_order(self) -> None:
        # Rz * Ry == q_yaw * q_pitch
        q = Quaternion.from_euler(0, 30, 45)
        r = m_mul(
            Quaternion.from_euler(0, 0, 45).to_matrix(),
            Quaternion.from_euler(0, 30, 0).to_matrix(),
        )
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(q.to_matrix()[i][j], r[i][j], places=9)

    def test_euler_roundtrip(self) -> None:
        roll, pitch, yaw = 10.0, 20.0, 30.0
        again = Quaternion.from_euler(roll, pitch, yaw).to_euler()
        self.assertAlmostEqual(again[0], roll, places=6)
        self.assertAlmostEqual(again[1], pitch, places=6)
        self.assertAlmostEqual(again[2], yaw, places=6)

    def test_gimbal_lock_pitch_only(self) -> None:
        _, pitch, _ = Quaternion.from_euler(0, 90, 0).to_euler()
        self.assertAlmostEqual(pitch, 90.0, places=6)

    def test_inverse_composes_to_identity(self) -> None:
        q = Quaternion.from_euler(12, -33, 77)
        r = m_mul(q.to_matrix(), q.inverse().to_matrix())
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(r[i][j], IDENTITY[i][j], places=9)

    def test_axis_angle(self) -> None:
        q = Quaternion.from_axis_angle((0, 0, 1), 180)
        v = q.rotate((1, 0, 0))
        self.assertAlmostEqual(v[0], -1.0, places=9)
        self.assertAlmostEqual(v[1], 0.0, places=9)

    def test_matrix_roundtrip(self) -> None:
        q = Quaternion.from_euler(-20, 15, 120)
        back = Quaternion.from_matrix(q.to_matrix()).to_matrix()
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(back[i][j], q.to_matrix()[i][j], places=9)


class TestNeedleKinematics(unittest.TestCase):
    def test_axis_of_identity_is_plus_z(self) -> None:
        self.assertEqual(needle_axis(IDENTITY), (0.0, 0.0, 1.0))

    def test_tip_straight_down(self) -> None:
        # 90 deg about X maps +Z to -Y
        r = Quaternion.from_euler(90, 0, 0).to_matrix()
        tip = tip_position((0.0, 0.0, 0.0), r, 100.0)
        self.assertAlmostEqual(tip[0], 0.0, places=9)
        self.assertAlmostEqual(tip[1], -100.0, places=9)
        self.assertAlmostEqual(tip[2], 0.0, places=9)

    def test_tip_with_entry_point(self) -> None:
        tip = tip_position((10.0, 20.0, 30.0), IDENTITY, 50.0)
        self.assertAlmostEqual(tip[2], 80.0, places=9)

    def test_elevation_azimuth(self) -> None:
        self.assertAlmostEqual(elevation_azimuth_deg((1, 0, 0))[0], 0.0, places=9)
        self.assertAlmostEqual(elevation_azimuth_deg((0, 0, 1))[0], 90.0, places=9)
        self.assertAlmostEqual(elevation_azimuth_deg((0, 1, 0))[1], 90.0, places=9)

    def test_deviation(self) -> None:
        self.assertAlmostEqual(deviation_deg((0, 0, 1), (0, 1, 0)), 90.0, places=9)
        tilted = (0.0, math.sin(math.radians(5)), math.cos(math.radians(5)))
        self.assertAlmostEqual(deviation_deg((0, 0, 1), tilted), 5.0, places=6)

    def test_tip_error_budget(self) -> None:
        # 1 deg at 150 mm ~= 2.6 mm lateral error
        self.assertAlmostEqual(predicted_tip_error_mm(150, 1.0), 2.618, places=3)
        self.assertAlmostEqual(predicted_tip_error_mm(150, 0.0), 0.0, places=9)

    def test_cone_angle(self) -> None:
        needle = (0.0, math.sin(math.radians(30)), math.cos(math.radians(30)))
        self.assertAlmostEqual(cone_angle_deg(needle), 30.0, places=6)


class TestMountingCalibration(unittest.TestCase):
    def test_recover_needle_axis_in_sensor(self) -> None:
        s_true = (0.2, -0.95, 0.15)
        s_true = tuple(v / math.sqrt(sum(c * c for c in s_true)) for v in s_true)
        r1 = Quaternion.from_euler(10, -20, 5).to_matrix()
        r2 = Quaternion.from_euler(-35, 40, 100).to_matrix()
        w1 = axis_world(r1, s_true)
        w2 = axis_world(r2, s_true)

        s, worst = needle_axis_in_sensor([(r1, w1), (r2, w2)])
        self.assertLess(deviation_deg(s, s_true), 1e-6)
        self.assertLess(worst, 1e-6)

    def test_rejects_single_pose(self) -> None:
        with self.assertRaises(ValueError):
            needle_axis_in_sensor([(IDENTITY, (0, 0, 1))])

    def test_relative_axis_cancels_reference(self) -> None:
        s = (0.0, 0.0, 1.0)
        r_ref = Quaternion.from_euler(0, 0, 30).to_matrix()
        r_now = Quaternion.from_euler(0, 10, 30).to_matrix()
        u = relative_axis(r_ref, r_now, s)
        # a 10 deg pitch change about the reference frame is pure elevation
        self.assertAlmostEqual(math.degrees(math.asin(u[2])), 80.0, places=6)

    def test_tip_from_axis(self) -> None:
        tip = tip_from_axis((0, 0, 0), (0, 0, -1), 120.0)
        self.assertAlmostEqual(tip[2], -120.0, places=9)


class TestFrames(unittest.TestCase):
    def test_frame_is_orthonormal(self) -> None:
        f = frame_from_two_vectors((1, 0, 0), (0, 1, 0))
        cols = [(f[0][i], f[1][i], f[2][i]) for i in range(3)]
        for i in range(3):
            self.assertAlmostEqual(v_norm(cols[i]), 1.0, places=12)
        for i in range(3):
            for j in range(i + 1, 3):
                self.assertAlmostEqual(v_dot(cols[i], cols[j]), 0.0, places=12)

    def test_align_recovers_known_rotation(self) -> None:
        r_true = Quaternion.from_euler(25, -40, 70).to_matrix()
        s1 = (0.3, 0.2, 0.93)
        s2 = (0.8, 0.1, -0.5)
        w1 = tuple(sum(r_true[i][k] * s1[k] for k in range(3)) for i in range(3))
        w2 = tuple(sum(r_true[i][k] * s2[k] for k in range(3)) for i in range(3))

        r = align_frames(s1, s2, w1, w2)
        self.assertLess(rotation_difference_deg(r, r_true), 1e-6)

    def test_align_rejects_parallel_poses(self) -> None:
        with self.assertRaises(ValueError):
            align_frames((0, 0, 1), (0, 0, 1.0001), (0, 0, 1), (0, 0, 1))

    def test_rotation_difference(self) -> None:
        r1 = IDENTITY
        r2 = Quaternion.from_axis_angle((0, 0, 1), 12.0).to_matrix()
        self.assertAlmostEqual(rotation_difference_deg(r1, r2), 12.0, places=6)


class TestPivotTransform(unittest.TestCase):
    def test_pivot_maps_to_anchor(self) -> None:
        q = Quaternion.from_euler(12, -25, 40)
        p = rotate_about((3.0, 4.0, 5.0), q, (3.0, 4.0, 5.0))
        self.assertTrue(all(close(p[i], 0.0) for i in range(3)), p)

    def test_explicit_anchor(self) -> None:
        q = Quaternion.from_axis_angle((0, 0, 1), 90.0)
        p = rotate_about((1.0, 0.0, 0.0), q, (0.0, 0.0, 0.0), anchor=(10.0, 20.0, 0.0))
        self.assertTrue(all(close(p[i], v) for i, v in enumerate((10.0, 21.0, 0.0))), p)

    def test_pivot_choice_only_changes_translation(self) -> None:
        q = Quaternion.from_axis_angle((0, 0, 1), 90.0)
        point = (1.0, 0.0, -10.0)
        about_hub = rotate_about(point, q, (0.0, 0.0, 0.0))
        about_tip = rotate_about(point, q, (0.0, 0.0, -10.0))
        self.assertTrue(
            all(close(about_hub[i], v) for i, v in enumerate((0.0, 1.0, -10.0))),
            about_hub,
        )
        # Tip mode places the tip at the origin, so the body shifts up by 10 mm.
        self.assertTrue(
            all(close(about_tip[i], v) for i, v in enumerate((0.0, 1.0, 0.0))),
            about_tip,
        )


if __name__ == "__main__":
    unittest.main()
