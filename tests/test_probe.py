"""Probe / capture analysis tests (no BLE needed)."""

from __future__ import annotations

import unittest

from wt901.probe import analyze_capture
from wt901.protocol import encode_combined, encode_split


class TestAnalyzeCapture(unittest.TestCase):
    def test_combined_stream(self) -> None:
        data = b"".join(encode_combined(0.0, 0.0, 1.0, 0, 0, 0, 0, 12.0, 90.0) for _ in range(100))
        result = analyze_capture(data, duration_s=1.0)
        self.assertEqual(result.frames_ok, 100)
        self.assertIn("0x61", result.format_name)
        self.assertAlmostEqual(result.rate_hz, 100.0, delta=0.1)
        self.assertAlmostEqual(result.accel_norm, 1.0, delta=0.01)
        self.assertEqual(len(result.samples), 5)

    def test_split_stream(self) -> None:
        data = b""
        for _ in range(50):
            data += encode_split(0x51, 0.0, 0.0, 1.0)
            data += encode_split(0x52, 0.0, 0.0, 0.0)
            data += encode_split(0x53, 0.0, 10.0, 0.0)
        result = analyze_capture(data, duration_s=0.5)
        self.assertEqual(result.frames_ok, 150)
        self.assertIn("split", result.format_name)
        self.assertEqual(set(result.frames_by_id), {0x51, 0x52, 0x53})

    def test_empty_capture(self) -> None:
        result = analyze_capture(b"", duration_s=3.0)
        self.assertEqual(result.frames_ok, 0)
        self.assertIsNone(result.rate_hz)
        self.assertTrue(any("no frame decoded" in n for n in result.notes))
        self.assertIsNotNone(result.as_text())

    def test_garbage_capture(self) -> None:
        result = analyze_capture(b"\xaa" * 500, duration_s=1.0)
        self.assertEqual(result.frames_ok, 0)
        self.assertGreater(result.bad_header + 0, -1)  # just must not crash

    def test_accel_norm_warning(self) -> None:
        data = b"".join(encode_combined(0.0, 0.0, 9.8, 0, 0, 0, 0, 0, 0) for _ in range(20))
        result = analyze_capture(data, duration_s=1.0)
        self.assertAlmostEqual(result.accel_norm, 9.8, delta=0.1)
        self.assertTrue(any("far from 1 g" in n for n in result.notes))

    def test_low_rate_warning(self) -> None:
        data = b"".join(encode_combined(0, 0, 1, 0, 0, 0, 0, 0, 0) for _ in range(3))
        result = analyze_capture(data, duration_s=5.0)
        self.assertTrue(any("Hz" in n for n in result.notes))

    def test_text_report_contains_key_lines(self) -> None:
        data = b"".join(encode_combined(0, 0, 1, 0, 0, 0, 1, 2, 3) for _ in range(10))
        text = analyze_capture(data, duration_s=1.0).as_text()
        self.assertIn("format:", text)
        self.assertIn("rate:", text)
        self.assertIn("first samples:", text)


if __name__ == "__main__":
    unittest.main()
