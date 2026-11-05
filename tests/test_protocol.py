"""Frame decoding tests - these are the tests that catch a wrong datasheet."""

from __future__ import annotations

import unittest

from wt901.protocol import (
    ID_ACCEL,
    ID_ANGLE,
    ID_COMBINED,
    ID_GYRO,
    Scales,
    WitParser,
    encode_combined,
    encode_split,
    parse_hex_text,
)


class TestCombinedFrame(unittest.TestCase):
    def setUp(self) -> None:
        self.parser = WitParser()

    def test_roundtrip(self) -> None:
        frame = encode_combined(0.5, -0.25, 1.0, 10.0, -20.0, 3.0, 4.0, 18.0, -170.0)
        self.assertEqual(len(frame), 20)
        self.assertEqual(frame[:2], b"\x55\x61")

        samples = self.parser.feed(frame)
        self.assertEqual(len(samples), 1)
        s = samples[0]
        self.assertAlmostEqual(s.ax, 0.5, places=3)
        self.assertAlmostEqual(s.ay, -0.25, places=3)
        self.assertAlmostEqual(s.az, 1.0, places=3)
        self.assertAlmostEqual(s.gx, 10.0, delta=0.02)
        self.assertAlmostEqual(s.pitch, 18.0, places=2)
        self.assertAlmostEqual(s.yaw, -170.0, places=2)

    def test_tiny_values_do_not_deadzone(self) -> None:
        frame = encode_combined(0.01, 0.0, 0.99, 0.0, 0.0, 0.0, 0.05, 0.05, 0.05)
        s = self.parser.feed(frame)[0]
        self.assertGreater(s.roll, 0.0)
        self.assertAlmostEqual(s.roll, 0.05, places=2)

    def test_negative_minimum_not_clipped(self) -> None:
        frame = encode_combined(-16.0, 0.0, 0.0, 0.0, 0.0, 0.0, -180.0, 0.0, 0.0)
        s = self.parser.feed(frame)[0]
        self.assertAlmostEqual(s.ax, -16.0, places=2)
        self.assertAlmostEqual(s.roll, -180.0, places=2)

    def test_split_across_two_notifications(self) -> None:
        frame = encode_combined(0, 0, 1, 0, 0, 0, 1, 2, 3)
        self.assertEqual(self.parser.feed(frame[:7]), [])
        out = self.parser.feed(frame[7:])
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].pitch, 2.0, places=2)

    def test_byte_by_byte(self) -> None:
        frame = encode_combined(0.1, 0.2, 0.97, 1, 2, 3, 5, 6, 7)
        collected = []
        for b in frame:
            collected.extend(self.parser.feed(bytes([b])))
        self.assertEqual(len(collected), 1)
        self.assertAlmostEqual(collected[0].yaw, 7.0, places=2)

    def test_disabled_combined_frames_are_ignored(self) -> None:
        parser = WitParser(combined_frames=False)
        frame = encode_combined(0, 0, 1, 0, 0, 0, 1, 2, 3)
        self.assertEqual(parser.feed(frame), [])


class TestSplitFrames(unittest.TestCase):
    def setUp(self) -> None:
        self.parser = WitParser()

    def test_three_frames(self) -> None:
        data = (
            encode_split(ID_ACCEL, 0.1, 0.2, 0.97)
            + encode_split(ID_GYRO, 1.0, 2.0, 3.0)
            + encode_split(ID_ANGLE, 10.0, -5.0, 90.0)
        )
        samples = self.parser.feed(data)
        self.assertEqual([s.kind for s in samples], ["accel", "gyro", "angle"])
        self.assertEqual(self.parser.stats.frames_ok, 3)
        self.assertAlmostEqual(samples[2].yaw, 90.0, places=2)

    def test_bad_checksum_is_rejected(self) -> None:
        frame = bytearray(encode_split(ID_ANGLE, 1.0, 2.0, 3.0))
        frame[-1] ^= 0xFF
        samples = self.parser.feed(bytes(frame))
        self.assertEqual(samples, [])
        self.assertEqual(self.parser.stats.bad_checksum, 1)

    def test_resync_after_garbage(self) -> None:
        payload = b"\x00\xff\x13\x37" + encode_split(ID_ANGLE, 1.0, 2.0, 3.0)
        samples = self.parser.feed(payload)
        self.assertEqual(len(samples), 1)
        self.assertGreater(self.parser.stats.discarded_bytes, 0)

    def test_bad_checksum_then_good_frame_in_same_chunk(self) -> None:
        bad = bytearray(encode_split(ID_ANGLE, 1.0, 2.0, 3.0))
        bad[-1] ^= 0xFF
        good = encode_split(ID_ANGLE, 4.0, 5.0, 6.0)
        samples = self.parser.feed(bytes(bad) + good)
        self.assertEqual(len(samples), 1)
        self.assertAlmostEqual(samples[0].roll, 4.0, places=2)

    def test_byte_by_byte(self) -> None:
        frame = encode_split(ID_GYRO, 100.0, -50.0, 25.0)
        out = []
        for b in frame:
            out.extend(self.parser.feed(bytes([b])))
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].gy, -50.0, delta=0.05)


class TestRobustness(unittest.TestCase):
    def test_no_header_stream_is_bounded(self) -> None:
        parser = WitParser(max_buffer=64)
        for _ in range(1000):
            parser.feed(b"\xaa\xbb\xcc")
        self.assertLessEqual(len(parser._buf), 1)

    def test_unknown_id_does_not_stall(self) -> None:
        parser = WitParser()
        data = b"\x55\x99" + b"\x00" * 30 + encode_split(ID_ANGLE, 7.0, 8.0, 9.0)
        samples = parser.feed(data)
        self.assertEqual(len(samples), 1)
        self.assertGreaterEqual(parser.stats.bad_header, 1)

    def test_stats_describe(self) -> None:
        parser = WitParser()
        parser.feed(encode_split(ID_ANGLE, 1, 2, 3))
        text = parser.stats.describe()
        self.assertIn("ok=1", text)
        self.assertIn("0x53", text)


class TestHexText(unittest.TestCase):
    def test_parse(self) -> None:
        self.assertEqual(parse_hex_text("55 61 0A"), b"\x55\x61\x0a")
        self.assertEqual(parse_hex_text("55,61\n0x0A"), b"\x55\x61\x0a")
        self.assertEqual(parse_hex_text("zz"), b"")


class TestScales(unittest.TestCase):
    def test_custom_range(self) -> None:
        parser = WitParser(scales=Scales(accel_g=8.0, gyro_dps=1000.0, angle_deg=180.0))
        frame = encode_combined(8.0, 0, 0, 0, 1000.0, 0, 0, 0, 0, scales=Scales(8.0, 1000.0))
        s = parser.feed(frame)[0]
        self.assertAlmostEqual(s.ax, 8.0, places=2)
        self.assertAlmostEqual(s.gy, 1000.0, delta=5.0)


if __name__ == "__main__":
    unittest.main()
