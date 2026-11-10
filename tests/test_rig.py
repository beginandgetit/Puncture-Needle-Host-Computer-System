"""Rig configuration and CAD-mode alignment tests."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from wt901.estimator import DOWN, NeedleTracker
from wt901.rig import Rig, align_model_point, tip_position_display
from wt901.pose import Quaternion, v_add, v_scale

HUB = (0.0, 18.38, 9.56)
AXIS = (0.0, 1.0, 0.0)
LEN = 176.35


def _near(a, b, tol=1e-6) -> bool:
    return all(abs(a[i] - b[i]) <= tol for i in range(3))


class Reading:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class TestRigConfig(unittest.TestCase):
    def test_defaults_match_cad(self) -> None:
        rig = Rig()
        self.assertEqual(rig.needle_axis_sensor, (0.0, 1.0, 0.0))
        self.assertEqual(rig.model_axis, (0.0, 1.0, 0.0))
        self.assertEqual(rig.model_right, (1.0, 0.0, 0.0))
        self.assertFalse(rig.invert_quaternion)
        self.assertEqual(rig.hub_mm, HUB)
        self.assertAlmostEqual(rig.needle_length_mm, LEN, places=3)
        self.assertTrue(rig.cad_mode)

    def test_missing_file_falls_back_to_defaults(self) -> None:
        rig = Rig.load(Path(tempfile.mkdtemp()) / "nope.json")
        self.assertEqual(rig, Rig())

    def test_save_and_load_roundtrip(self) -> None:
        rig = Rig(
            needle_length_mm=200.0,
            cad_mode=False,
            model_right=(0.0, 0.0, 1.0),
            invert_quaternion=True,
        )
        path = Path(tempfile.mkdtemp()) / "rig.json"
        rig.save(path)
        self.assertTrue(path.exists())
        self.assertEqual(Rig.load(path), rig)


class TestModelAlignment(unittest.TestCase):
    def test_hub_maps_to_origin(self) -> None:
        p = align_model_point(HUB, AXIS, HUB, DOWN)
        self.assertTrue(_near(p, (0.0, 0.0, 0.0)), p)

    def test_tip_maps_along_display_axis(self) -> None:
        tip = v_add(HUB, v_scale(AXIS, LEN))
        p = align_model_point(tip, AXIS, HUB, DOWN)
        self.assertTrue(_near(p, (0.0, 0.0, -LEN)), p)

    def test_tip_helper_matches(self) -> None:
        rig = Rig()
        p = tip_position_display(rig, DOWN)
        self.assertTrue(_near(p, (0.0, 0.0, -LEN)), p)

    def test_model_placement_maps_axis_down(self) -> None:
        rig = Rig()
        q = rig.model_placement()
        p = q.rotate(rig.model_axis)
        self.assertTrue(_near(p, DOWN, 1e-9), p)

    def test_alignment_keeps_distances(self) -> None:
        q = Quaternion.from_two_vectors(AXIS, (0.3, 0.4, -0.8660254))
        v = (10.0, 18.38, 9.56)
        d0 = (v[0] - HUB[0], v[1] - HUB[1], v[2] - HUB[2])
        rotated = q.rotate(d0)
        n0 = sum(x * x for x in d0) ** 0.5
        n1 = sum(x * x for x in rotated) ** 0.5
        self.assertAlmostEqual(n0, n1, places=6)


class TestTrackerWithCadAxis(unittest.TestCase):
    def test_reference_is_zero_then_tilt_shows_deviation(self) -> None:
        tracker = NeedleTracker(needle_axis_sensor=AXIS, smoothing_s=0.0)
        tracker.update(Reading(0, 0, 0), now=1.0)
        pose = tracker.update(Reading(0, 0, 0), now=1.1)
        self.assertAlmostEqual(pose.deviation_deg, 0.0, places=4)
        moved = tracker.update(Reading(12, 0, 0), now=1.2)
        self.assertGreater(moved.deviation_deg, 5.0)


if __name__ == "__main__":
    unittest.main()
