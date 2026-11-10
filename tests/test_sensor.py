"""SDK tests: sync facade, async facade, calibration, reconnect, fail-safe."""

from __future__ import annotations

import asyncio
import os
import time
import unittest

from wt901 import (
    AsyncWT901,
    ConfigError,
    ConnectionTimeout,
    NoDataError,
    WT901,
    WT901Config,
)
from wt901.errors import NotStartedError
from wt901.protocol import encode_combined, encode_quaternion, encode_split


class FakeTransport:
    """Emits a fixed payload at a fixed rate; optionally drops the link."""

    def __init__(self, payload: bytes, *, hz: float = 100.0, fail_after: float | None = None):
        self.payload = payload
        self.hz = hz
        self.fail_after = fail_after
        self.runs = 0
        self.device = None
        self.notify_uuid = "fake-notify"
        self.sent = []

    async def run(self, on_bytes) -> None:
        self.runs += 1
        started = time.monotonic()
        period = 1.0 / self.hz
        while True:
            if self.fail_after is not None and time.monotonic() - started > self.fail_after:
                raise ConnectionError("fake link drop")
            on_bytes(self.payload)
            await asyncio.sleep(period)

    async def send(self, data: bytes) -> None:
        self.sent.append(data)

    async def close(self) -> None:
        return None


def payload(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
            ax: float = 0.0, ay: float = 0.0, az: float = 1.0,
            gx: float = 0.0, gy: float = 0.0, gz: float = 0.0) -> bytes:
    return encode_combined(ax, ay, az, gx, gy, gz, roll, pitch, yaw)


def config(transport, **kwargs) -> WT901Config:
    defaults = dict(
        transport=transport,
        connect_timeout_s=3.0,
        reconnect_delay_s=0.05,
        reconnect_max_s=0.1,
    )
    defaults.update(kwargs)
    return WT901Config(**defaults)


class TestAsync(unittest.IsolatedAsyncioTestCase):
    async def test_read_and_status(self) -> None:
        cfg = config(FakeTransport(payload(pitch=17.0)))
        async with AsyncWT901(cfg) as imu:
            reading = await imu.read(timeout=2.0)
            self.assertAlmostEqual(reading.pitch, 17.0, delta=0.1)
            self.assertAlmostEqual(reading.az, 1.0, delta=0.01)
            self.assertGreater(reading.seq, 0)
            self.assertGreater(reading.t_wall, 1_600_000_000)

            # The Windows monotonic clock ticks coarsely, so give it real
            # history before asserting on the measured rate.
            await asyncio.sleep(0.35)
            status = imu.status
            self.assertEqual(status.state.value, "connected")
            self.assertGreater(status.frames, 0)
            self.assertGreaterEqual(status.rate_hz, 1.0)
            self.assertLess(status.last_reading_age_s, 1.0)

    async def test_stream(self) -> None:
        cfg = config(FakeTransport(payload(pitch=5.0)))
        async with AsyncWT901(cfg) as imu:
            seen = []
            async for reading in imu.stream():
                seen.append(reading)
                if len(seen) >= 3:
                    break
            self.assertEqual(len(seen), 3)

    async def test_timeout_raises_nodata(self) -> None:
        cfg = config(FakeTransport(b"", hz=20.0), connect_timeout_s=1.0)
        imu = AsyncWT901(cfg)
        await imu.start()
        try:
            with self.assertRaises(NoDataError):
                await imu.read(timeout=0.2)
        finally:
            await imu.stop()

    async def test_wait_connected_times_out(self) -> None:
        cfg = config(FakeTransport(b"", hz=20.0))
        imu = AsyncWT901(cfg)
        await imu.start()
        try:
            with self.assertRaises(ConnectionTimeout):
                await imu.wait_connected(0.3)
        finally:
            await imu.stop()

    async def test_reconnects_after_drop(self) -> None:
        transport = FakeTransport(payload(pitch=1.0), fail_after=0.25)
        imu = AsyncWT901(config(transport))
        await imu.start()
        try:
            await imu.read(timeout=2.0)
            await asyncio.sleep(0.7)
            self.assertGreaterEqual(imu.status.reconnects, 1)
            self.assertGreaterEqual(transport.runs, 2)
            again = await imu.read(timeout=2.0)
            self.assertIsNotNone(again)
        finally:
            await imu.stop()

    async def test_gyro_bias_calibration(self) -> None:
        cfg = config(FakeTransport(payload(gx=10.0, gy=-2.0, gz=0.5)))
        async with AsyncWT901(cfg) as imu:
            bias = await imu.calibrate_gyro_bias(0.3)
            self.assertAlmostEqual(bias[0], 10.0, delta=0.2)
            await asyncio.sleep(0.15)
            self.assertAlmostEqual(imu.latest.gx, 0.0, delta=0.3)
            self.assertAlmostEqual(imu.latest.gy, 0.0, delta=0.3)

    async def test_set_output_rate_writes_register(self) -> None:
        from wt901.ble import wit_frame

        transport = FakeTransport(payload())
        async with AsyncWT901(config(transport)) as imu:
            await imu.set_output_rate(50, save=True)
        self.assertEqual(transport.sent[0], wit_frame(0x03, 0x08))
        self.assertEqual(transport.sent[1], wit_frame(0x00, 0x0000))

    async def test_zero_here(self) -> None:
        cfg = config(FakeTransport(payload(roll=3.0, pitch=20.0)))
        async with AsyncWT901(cfg) as imu:
            await imu.zero_here(samples=10)
            await asyncio.sleep(0.15)
            self.assertAlmostEqual(imu.latest.pitch, 0.0, delta=0.3)
            self.assertAlmostEqual(imu.latest.roll, 0.0, delta=0.3)

    async def test_save_and_load_calibration(self) -> None:
        path = os.path.join(os.path.dirname(__file__), "_artifacts", "cal.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cfg = config(FakeTransport(payload()))
        async with AsyncWT901(cfg) as imu:
            imu.set_gyro_bias(0.5, 0.6, 0.7)
            imu.set_angle_offset(1.0, 2.0, 3.0)
            imu.save_calibration(path)

            other = config(FakeTransport(payload()))
            async with AsyncWT901(other) as imu2:
                imu2.load_calibration(path)
                self.assertEqual(imu2.gyro_bias, (0.5, 0.6, 0.7))
                self.assertEqual(imu2.angle_offset, (1.0, 2.0, 3.0))
        os.remove(path)

    async def test_listener_receives_readings(self) -> None:
        cfg = config(FakeTransport(payload()))
        got = []
        async with AsyncWT901(cfg) as imu:
            imu.add_listener(got.append)
            await asyncio.sleep(0.2)
        self.assertGreater(len(got), 0)

    async def test_emit_on_any_for_split_frames(self) -> None:
        # Only accel frames: with the default emit_on="attitude" nothing is
        # published, with "any" every frame is.
        only_accel = encode_split(0x51, 0.0, 0.0, 1.0)
        cfg = config(FakeTransport(only_accel), emit_on="any")
        async with AsyncWT901(cfg) as imu:
            reading = await imu.read(timeout=2.0)
            self.assertIsNotNone(reading)


class TestSync(unittest.TestCase):
    def test_context_manager_read(self) -> None:
        cfg = config(FakeTransport(payload(pitch=-12.0)))
        with WT901(cfg) as imu:
            reading = imu.read(timeout=3.0)
            self.assertAlmostEqual(reading.pitch, -12.0, delta=0.1)
            self.assertGreater(imu.status.frames, 0)
            self.assertEqual(imu.state.value, "connected")

    def test_stream_and_stop(self) -> None:
        cfg = config(FakeTransport(payload()))
        imu = WT901(cfg).start()
        try:
            count = 0
            for _ in imu.stream():
                count += 1
                if count >= 3:
                    break
            self.assertEqual(count, 3)
        finally:
            imu.stop()
        self.assertIsNone(imu.latest)

    def test_read_without_start_raises(self) -> None:
        imu = WT901(config(FakeTransport(payload())))
        with self.assertRaises(NotStartedError):
            imu.read(timeout=1.0)

    def test_calibration_through_sync_facade(self) -> None:
        cfg = config(FakeTransport(payload(gx=4.0)))
        with WT901(cfg) as imu:
            bias = imu.calibrate_gyro_bias(0.3)
            self.assertAlmostEqual(bias[0], 4.0, delta=0.3)

    def test_stop_is_idempotent(self) -> None:
        imu = WT901(config(FakeTransport(payload()))).start()
        imu.stop()
        imu.stop()


class TestConfigValidation(unittest.TestCase):
    def test_missing_target(self) -> None:
        # A bare config is valid: it falls back to scanning for "WT901".
        WT901Config().validate()
        with self.assertRaises(ConfigError):
            WT901Config(name_contains="", address=None).validate()

    def test_bad_emit_on(self) -> None:
        with self.assertRaises(ConfigError):
            WT901Config(transport=object(), emit_on="sometimes").validate()

    def test_bad_reconnect_window(self) -> None:
        with self.assertRaises(ConfigError):
            WT901Config(transport=object(), reconnect_delay_s=5, reconnect_max_s=1).validate()

    def test_sdk_constructors_validate(self) -> None:
        with self.assertRaises(ConfigError):
            AsyncWT901(WT901Config(name_contains="", address=None))
        with self.assertRaises(ConfigError):
            WT901(WT901Config(name_contains="", address=None))


class TestQuaternionPipeline(unittest.IsolatedAsyncioTestCase):
    async def test_quaternion_only_stream_publishes_pose(self) -> None:
        cfg = config(FakeTransport(encode_quaternion(0.5, -0.5, 0.25, -0.25), hz=50.0))
        async with AsyncWT901(cfg) as imu:
            reading = await imu.read(timeout=2.0)
            self.assertTrue(reading.has_quaternion)
            self.assertFalse(reading.has_attitude)
            self.assertAlmostEqual(reading.qw, 0.5, places=5)
            self.assertAlmostEqual(reading.qx, -0.5, places=5)
            self.assertAlmostEqual(reading.qy, 0.25, places=5)
            self.assertAlmostEqual(reading.qz, -0.25, places=5)
            self.assertEqual(
                tuple(round(v, 5) for v in reading.quaternion),
                (0.5, -0.5, 0.25, -0.25),
            )
            self.assertTrue(reading.as_dict()["has_quaternion"])


class TestStateQuaternion(unittest.TestCase):
    def test_buffer_merges_quaternion_frames(self) -> None:
        from wt901.model import ImuSample
        from wt901.state import TelemetryBuffer

        buf = TelemetryBuffer()
        buf.update(
            ImuSample(
                kind="quaternion",
                t_mono=1.0,
                qw=0.5,
                qx=-0.5,
                qy=0.25,
                qz=-0.25,
            )
        )
        snap = buf.snapshot(2.0)
        self.assertEqual(snap.quaternion, (0.5, -0.5, 0.25, -0.25))
        self.assertAlmostEqual(snap.age_quaternion(2.0), 1.0)
        self.assertIsNone(snap.attitude)

    def test_snapshot_from_reading_carries_quaternion(self) -> None:
        import time

        from wt901.sensor import ImuReading
        from wt901.state import Snapshot

        reading = ImuReading(
            t_mono=time.monotonic(),
            t_wall=time.time(),
            seq=1,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            ax=0.0,
            ay=0.0,
            az=1.0,
            gx=0.0,
            gy=0.0,
            gz=0.0,
            has_attitude=False,
            qw=0.5,
            qx=-0.5,
            qy=0.25,
            qz=-0.25,
        )
        snap = Snapshot.from_reading(reading)
        self.assertEqual(snap.quaternion, (0.5, -0.5, 0.25, -0.25))
        self.assertIsNone(snap.attitude)


if __name__ == "__main__":
    unittest.main()
