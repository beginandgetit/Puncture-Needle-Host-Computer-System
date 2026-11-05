"""Controller behaviour: hysteresis, fail-safe, slew limiting, tilt cut."""

from __future__ import annotations

import unittest

from wt901.controller import FWD, REV, STOP, AttitudeController, ControlConfig
from wt901.state import Snapshot, TelemetryBuffer
from wt901.model import ImuSample


def snap(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
         now: float = 100.0, age: float = 0.0) -> Snapshot:
    return Snapshot(
        t_mono=now,
        accel=None,
        gyro=None,
        attitude=(roll, pitch, yaw),
        attitude_t=now - age,
        accel_t=now - age,
        gyro_t=now - age,
    )


def drive(ctrl: AttitudeController, pitch: float, *, now: float, steps: int = 60,
          dt: float = 0.02, roll: float = 0.0):
    """Run enough control ticks that the slew-limited command reaches target."""

    cmd = None
    for _ in range(steps):
        now += dt
        cmd = ctrl.step(snap(roll=roll, pitch=pitch, now=now), now)
    return cmd


class TestThresholds(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = AttitudeController(ControlConfig(threshold_deg=15.0, hysteresis_deg=3.0))

    def test_below_threshold_is_stop(self) -> None:
        cmd = drive(self.ctrl, 5.0, now=100.0)
        self.assertEqual(cmd.name, STOP)
        self.assertEqual(cmd.value, 0.0)
        self.assertTrue(cmd.valid)

    def test_positive_crossing_is_fwd(self) -> None:
        cmd = drive(self.ctrl, 20.0, now=100.0)
        self.assertEqual(cmd.name, FWD)
        self.assertAlmostEqual(cmd.value, 1.0, places=6)

    def test_negative_crossing_is_rev(self) -> None:
        cmd = drive(self.ctrl, -20.0, now=100.0)
        self.assertEqual(cmd.name, REV)
        self.assertAlmostEqual(cmd.value, -1.0, places=6)

    def test_hysteresis_holds_at_13_degrees(self) -> None:
        drive(self.ctrl, 20.0, now=100.0)
        cmd = drive(self.ctrl, 13.0, now=110.0)
        self.assertEqual(cmd.name, FWD, "13 deg is inside the 15-3 hold band")

    def test_release_below_12_degrees(self) -> None:
        drive(self.ctrl, 20.0, now=100.0)
        cmd = drive(self.ctrl, 11.0, now=110.0)
        self.assertEqual(cmd.name, STOP)

    def test_exactly_at_threshold_triggers(self) -> None:
        cmd = drive(self.ctrl, 15.0, now=100.0)
        self.assertEqual(cmd.name, FWD)


class TestFailSafe(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = AttitudeController(ControlConfig(stale_timeout_s=0.5))

    def test_no_attitude_is_stop_invalid(self) -> None:
        s = Snapshot(now := 100.0, None, None, None, 0.0, 0.0, 0.0)
        cmd = self.ctrl.step(s, now)
        self.assertEqual(cmd.name, STOP)
        self.assertFalse(cmd.valid)
        self.assertEqual(cmd.reason, "no_attitude")

    def test_stale_data_is_stop_invalid(self) -> None:
        cmd = self.ctrl.step(snap(pitch=30.0, now=100.0, age=0.9), 100.0)
        self.assertEqual(cmd.name, STOP)
        self.assertFalse(cmd.valid)
        self.assertIn("stale", cmd.reason)

    def test_recovers_after_fresh_data(self) -> None:
        self.ctrl.step(snap(pitch=30.0, now=100.0, age=0.9), 100.0)
        cmd = drive(self.ctrl, 30.0, now=101.0)
        self.assertEqual(cmd.name, FWD)
        self.assertTrue(cmd.valid)

    def test_tilt_limit_cuts_output(self) -> None:
        cmd = drive(self.ctrl, 20.0, now=100.0, roll=75.0)
        self.assertEqual(cmd.name, STOP)
        self.assertFalse(cmd.valid)
        self.assertIn("tilt_limit", cmd.reason)


class TestSlew(unittest.TestCase):
    def test_ramps_instead_of_stepping(self) -> None:
        ctrl = AttitudeController(ControlConfig(slew_per_s=2.0))
        t = 100.0
        ctrl.step(snap(pitch=20.0, now=t), t)       # first call: dt unknown -> dt=0
        t += 0.1
        cmd = ctrl.step(snap(pitch=20.0, now=t), t)  # 0.1 s * 2.0 = 0.2
        self.assertAlmostEqual(cmd.value, 0.2, places=6)
        self.assertEqual(cmd.name, FWD)

    def test_reaches_full_scale(self) -> None:
        ctrl = AttitudeController(ControlConfig(slew_per_s=2.0))
        cmd = drive(ctrl, 20.0, now=100.0, steps=60, dt=0.02)  # 1.2 s
        self.assertAlmostEqual(cmd.value, 1.0, places=6)

    def test_stop_also_ramps_down(self) -> None:
        ctrl = AttitudeController(ControlConfig(slew_per_s=2.0))
        t = 100.0
        for _ in range(60):
            t += 0.02
            ctrl.step(snap(pitch=20.0, now=t), t)
        t += 0.02
        cmd = ctrl.step(snap(pitch=0.0, now=t), t)
        self.assertAlmostEqual(cmd.value, 0.96, places=6)  # 0.02 s * 2.0 back
        self.assertEqual(cmd.name, FWD)


class TestAxisSelection(unittest.TestCase):
    def test_roll_axis(self) -> None:
        ctrl = AttitudeController(ControlConfig(axis="roll"))
        cmd = drive(ctrl, 0.0, roll=25.0, now=100.0)
        self.assertEqual(cmd.name, FWD)

    def test_invalid_config_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ControlConfig(axis="x").validate()
        with self.assertRaises(ValueError):
            ControlConfig(hysteresis_deg=20.0).validate()


class TestTelemetryBuffer(unittest.TestCase):
    def test_merge_split_frames(self) -> None:
        buf = TelemetryBuffer()
        buf.update(ImuSample(kind="accel", t_mono=1.0, ax=0, ay=0, az=1))
        buf.update(ImuSample(kind="gyro", t_mono=1.01, gx=1, gy=2, gz=3))
        buf.update(ImuSample(kind="angle", t_mono=1.02, roll=1, pitch=2, yaw=3))
        s = buf.snapshot(1.5)
        self.assertEqual(s.attitude, (1.0, 2.0, 3.0))
        self.assertEqual(s.accel, (0.0, 0.0, 1.0))
        self.assertAlmostEqual(s.age_attitude(1.5), 0.48)

    def test_reset_drops_values(self) -> None:
        buf = TelemetryBuffer()
        buf.update(ImuSample(kind="angle", t_mono=1.0, roll=1, pitch=2, yaw=3))
        buf.reset()
        self.assertIsNone(buf.snapshot(2.0).attitude)


if __name__ == "__main__":
    unittest.main()
