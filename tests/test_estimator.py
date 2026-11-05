"""NeedleTracker tests: reference frame, tilt decomposition, plan, smoothing."""

from __future__ import annotations

import math
import unittest

from wt901.estimator import DOWN, NeedleTracker, tilt_components
from wt901.pose import Quaternion, deviation_deg


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
        self.assertEqual(d["valid"], True)


if __name__ == "__main__":
    unittest.main()
