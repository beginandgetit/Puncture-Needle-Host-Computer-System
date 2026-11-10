"""NeedleTracker tests: reference frame, tilt decomposition, plan, smoothing."""

from __future__ import annotations

import math
import unittest

from wt901.estimator import (
    DOWN,
    NeedleTracker,
    direction_word,
    plan_deviation,
    tilt_components,
)
from wt901.pose import Quaternion, deviation_deg, v_angle_deg


class Reading:
    """Minimal stand-in for ``wt901.ImuReading``."""

    def __init__(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class TestTiltComponents(unittest.TestCase):
    def test_straight_down(self) -> None:
        total, x, y = tilt_components(DOWN)
        self.assertAlmostEqual(total, 0.0, places=9)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)

    def test_pure_x_tilt(self) -> None:
        axis = (math.sin(math.radians(10)), 0.0, -math.cos(math.radians(10)))
        total, x, y = tilt_components(axis)
        self.assertAlmostEqual(total, 10.0, places=6)
        self.assertAlmostEqual(x, 10.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_total_is_geometric_sum_near_down(self) -> None:
        axis = (math.sin(math.radians(3)), math.sin(math.radians(4)), -0.999)
        total, x, y = tilt_components(axis)
        self.assertAlmostEqual(total, math.hypot(x, y), delta=0.05)


class TestTracker(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = NeedleTracker(smoothing_s=0.0)

    def test_first_reading_is_zero(self) -> None:
        pose = self.tracker.update(Reading(10, 20, 30), now=1.0)
        self.assertAlmostEqual(pose.tilt_deg, 0.0, places=6)
        self.assertAlmostEqual(pose.deviation_deg, 0.0, places=6)
        self.assertTrue(self.tracker.has_reference)

    def test_tilt_after_reference(self) -> None:
        self.tracker.update(Reading(0, 0, 0), now=1.0)
        pose = self.tracker.update(Reading(10, 0, 0), now=1.1)
        self.assertAlmostEqual(pose.tilt_deg, 10.0, places=5)
        # A rotation about the sensor X axis tilts forward/back, i.e. into Y.
        self.assertAlmostEqual(pose.tilt_x_deg, 0.0, places=5)
        self.assertAlmostEqual(abs(pose.tilt_y_deg), 10.0, places=5)
        self.assertAlmostEqual(pose.deviation_deg, 10.0, places=5)

    def test_plan_is_captured_direction(self) -> None:
        self.tracker.update(Reading(0, 0, 0), now=1.0)
        self.tracker.update(Reading(8, 0, 0), now=1.1)
        self.tracker.plan_from_current()
        pose = self.tracker.update(Reading(8, 0, 0), now=1.2)
        self.assertAlmostEqual(pose.deviation_deg, 0.0, places=5)

    def test_deviation_is_relative_to_plan(self) -> None:
        self.tracker.update(Reading(0, 0, 0), now=1.0)
        self.tracker.update(Reading(8, 0, 0), now=1.1)
        self.tracker.plan_from_current()
        pose = self.tracker.update(Reading(5, 0, 0), now=1.2)
        self.assertAlmostEqual(pose.deviation_deg, 3.0, places=5)

    def test_heading_change_does_not_change_tilt(self) -> None:
        # Rotating about the vertical axis keeps the angle to vertical, so the
        # displayed tilt must not change even though the direction does.
        self.tracker.update(Reading(0, 0, 0), now=1.0)
        a = self.tracker.update(Reading(12, 0, 0), now=1.1)
        b = self.tracker.update(Reading(12, 0, 90), now=1.2)
        self.assertAlmostEqual(a.tilt_deg, b.tilt_deg, places=5)

    def test_reference_reset_zeroes_everything(self) -> None:
        self.tracker.update(Reading(0, 0, 0), now=1.0)
        self.tracker.update(Reading(15, 0, 0), now=1.1)
        self.tracker.capture_reference(Quaternion.from_euler(15, 0, 0))
        pose = self.tracker.update(Reading(15, 0, 0), now=1.2)
        self.assertAlmostEqual(pose.tilt_deg, 0.0, places=5)

    def test_vertical_calibration_recovers_axis(self) -> None:
        # Simulate a mount where the needle is along the sensor's +X instead of +Z.
        q = Quaternion.from_two_vectors((1, 0, 0), DOWN)
        s = self.tracker.calibration_from_vertical(q)
        self.assertAlmostEqual(deviation_deg(s, (1, 0, 0)), 0.0, places=6)

    def test_smoothing_reduces_step(self) -> None:
        smooth = NeedleTracker(smoothing_s=0.5)
        smooth.update(Reading(0, 0, 0), now=1.0)
        pose = smooth.update(Reading(20, 0, 0), now=1.02)  # 20 ms after the step
        self.assertLess(pose.tilt_deg, 20.0)
        self.assertGreater(pose.tilt_deg, 0.0)

    def test_axis_is_unit(self) -> None:
        pose = self.tracker.update(Reading(3, 4, 5), now=1.0)
        length = math.sqrt(sum(v * v for v in pose.axis))
        self.assertAlmostEqual(length, 1.0, places=9)

    def test_dict_export(self) -> None:
        pose = self.tracker.update(Reading(1, 2, 3), now=1.0)
        d = pose.as_dict()
        self.assertIn("tilt_deg", d)


def _dir(degrees_x: float, degrees_y: float):
    """Unit vector tilted from straight down by the given angles."""

    rx, ry = math.radians(degrees_x), math.radians(degrees_y)
    return (math.sin(rx), math.sin(ry) * math.cos(rx), -math.cos(rx) * math.cos(ry))


class TestPlanDeviation(unittest.TestCase):
    def test_phi_when_tilted_right(self) -> None:
        phi, psi = plan_deviation(_dir(10, 0), DOWN)
        self.assertAlmostEqual(phi, 10.0, places=4)
        self.assertAlmostEqual(psi, 0.0, places=4)

    def test_psi_when_tilted_forward(self) -> None:
        phi, psi = plan_deviation(_dir(0, 10), DOWN)
        self.assertAlmostEqual(phi, 0.0, places=4)
        self.assertAlmostEqual(psi, 10.0, places=4)

    def test_negative_directions(self) -> None:
        phi, psi = plan_deviation(_dir(-8, -6), DOWN)
        self.assertAlmostEqual(phi, -8.0, delta=0.1)
        self.assertAlmostEqual(psi, -6.0, delta=0.1)

    def test_total_matches_quadrature_for_small_angles(self) -> None:
        axis = _dir(3.0, 4.0)
        phi, psi = plan_deviation(axis, DOWN)
        self.assertAlmostEqual(math.hypot(phi, psi), deviation_deg(axis, DOWN), delta=0.02)

    def test_aligned_gives_zero(self) -> None:
        phi, psi = plan_deviation(DOWN, DOWN)
        self.assertAlmostEqual(phi, 0.0, places=6)
        self.assertAlmostEqual(psi, 0.0, places=6)


class TestDirectionWord(unittest.TestCase):
    def test_aligned(self) -> None:
        self.assertEqual(direction_word(0.0, 0.0), "对准")

    def test_sectors(self) -> None:
        self.assertEqual(direction_word(0.0, 3.0), "上")
        self.assertEqual(direction_word(0.0, -3.0), "下")
        self.assertEqual(direction_word(3.0, 0.0), "右")
        self.assertEqual(direction_word(-3.0, 0.0), "左")
        self.assertEqual(direction_word(2.0, 2.0), "右上")
        self.assertEqual(direction_word(-2.0, -2.0), "左下")

    def test_pose_carries_phi_psi(self) -> None:
        tracker = NeedleTracker(smoothing_s=0.0)
        tracker.update(Reading(0, 0, 0), now=1.0)
        # pitch rotates the needle towards the X (left/right) axis
        pose = tracker.update(Reading(0, 9, 0), now=1.1)
        self.assertAlmostEqual(abs(pose.deviation_x_deg), 9.0, delta=0.1)
        self.assertLess(abs(pose.deviation_y_deg), 0.1)
        self.assertIn(pose.direction, ("左", "右"))
        self.assertIn("deviation_x_deg", pose.as_dict())
        self.assertTrue(pose.as_dict()["valid"])


class QuatReading:
    """Reading that carries a device quaternion instead of Euler angles."""

    def __init__(self, q: Quaternion):
        self.quaternion = (q.w, q.x, q.y, q.z)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


class TestQuaternionInput(unittest.TestCase):
    def test_quaternion_path_matches_euler_path(self) -> None:
        euler = NeedleTracker(smoothing_s=0.0)
        quat = NeedleTracker(smoothing_s=0.0)
        for i, (roll, pitch, yaw) in enumerate(
            ((0, 0, 0), (10, 0, 0), (10, 5, 20), (0, 0, 90))
        ):
            now = 1.0 + i * 0.1
            a = euler.update(Reading(roll, pitch, yaw), now=now)
            b = quat.update(QuatReading(Quaternion.from_euler(roll, pitch, yaw)), now=now)
            self.assertAlmostEqual(a.tilt_deg, b.tilt_deg, places=6)
            self.assertAlmostEqual(a.deviation_deg, b.deviation_deg, places=6)

    def test_inverted_convention_is_compensated(self) -> None:
        normal = NeedleTracker(smoothing_s=0.0)
        inverted = NeedleTracker(smoothing_s=0.0, invert_quaternion=True)
        q = Quaternion.from_euler(10, 20, 30)
        normal.update(QuatReading(q), now=1.0)
        inverted.update(QuatReading(q.inverse()), now=1.0)
        a = normal.update(QuatReading(q), now=2.0)
        b = inverted.update(QuatReading(q.inverse()), now=2.0)
        self.assertAlmostEqual(a.tilt_deg, b.tilt_deg, places=6)

    def test_zero_quaternion_falls_back_to_euler(self) -> None:
        tracker = NeedleTracker(smoothing_s=0.0)
        pose = tracker.update(QuatReading(Quaternion(0.0, 0.0, 0.0, 0.0)), now=1.0)
        self.assertAlmostEqual(pose.tilt_deg, 0.0, places=6)
        self.assertTrue(tracker.has_reference)

    def test_model_rotation_is_emitted(self) -> None:
        tracker = NeedleTracker(smoothing_s=0.0)
        tracker.update(QuatReading(Quaternion.identity()), now=1.0)
        pose = tracker.update(
            QuatReading(Quaternion.from_axis_angle((1, 0, 0), 12)), now=1.1
        )
        self.assertIsNotNone(pose.model_rotation)
        # The model's needle axis must agree with the reported needle axis.
        self.assertLess(
            v_angle_deg(pose.model_rotation.rotate((0.0, 0.0, -1.0)), pose.axis),
            1e-6,
        )

    def test_roll_about_needle_turns_the_model_handle(self) -> None:
        tracker = NeedleTracker(
            needle_axis_sensor=(0, 1, 0),
            model_axis=(0, 1, 0),
            model_right=(1, 0, 0),
            smoothing_s=0.0,
        )
        tracker.update(QuatReading(Quaternion.identity()), now=1.0)
        pose = tracker.update(
            QuatReading(Quaternion.from_axis_angle((0, 1, 0), 30)), now=1.1
        )
        # Axis unchanged, handle-right rotated by the roll angle.
        self.assertLess(v_angle_deg(pose.axis, (0.0, 0.0, -1.0)), 1e-6)
        right0 = tracker.model_placement.rotate((1.0, 0.0, 0.0))
        right1 = pose.model_rotation.rotate((1.0, 0.0, 0.0))
        self.assertAlmostEqual(v_angle_deg(right0, right1), 30.0, places=6)

    def test_two_pose_calibration_updates_axis(self) -> None:
        tracker = NeedleTracker(smoothing_s=0.0)
        axis = (0.2, -0.9, 0.3)
        q1 = Quaternion.from_two_vectors(axis, DOWN)
        q2 = Quaternion.from_two_vectors(axis, (1.0, 0.0, 0.0))
        recovered, worst = tracker.calibration_from_two_poses(
            [(q1.to_matrix(), DOWN), (q2.to_matrix(), (1.0, 0.0, 0.0))]
        )
        self.assertLess(worst, 0.01)
        self.assertLess(v_angle_deg(recovered, axis), 1e-6)


if __name__ == "__main__":
    unittest.main()
