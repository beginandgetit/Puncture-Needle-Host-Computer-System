"""End-to-end pipeline tests: no Bluetooth, no hardware."""

from __future__ import annotations

import asyncio
import os
import unittest

from wt901.app import App, AppConfig
from wt901.recorder import CsvRecorder
from wt901.simulator import SimulatedTransport
from wt901.sinks import ListSink


async def _run_for(seconds: float, app: App) -> None:
    try:
        await asyncio.wait_for(app.run(), timeout=seconds)
    except (asyncio.TimeoutError, asyncio.CancelledError):
        pass


class TestPipeline(unittest.TestCase):
    def test_simulated_combined_stream_produces_both_directions(self) -> None:
        sink = ListSink()
        app = App(
            SimulatedTransport(hz=100.0, period_s=1.0),
            cfg=AppConfig(control_hz=50.0, stats_interval_s=1e9, print_data=False),
            sinks=sink,
        )
        asyncio.run(_run_for(1.4, app))
        names = sink.names
        self.assertIn("FWD", names)
        self.assertIn("REV", names)
        self.assertEqual(names[-1], "STOP", "shutdown must always leave the plant stopped")

    def test_simulated_split_stream(self) -> None:
        sink = ListSink()
        app = App(
            SimulatedTransport(hz=100.0, period_s=1.0, split=True),
            cfg=AppConfig(control_hz=50.0, stats_interval_s=1e9, print_data=False),
            sinks=sink,
        )
        asyncio.run(_run_for(1.4, app))
        self.assertIn("FWD", sink.names)

    def test_csv_recording(self) -> None:
        # Write inside the repo: some Windows environments deny access to the
        # platform temp directory, which would make this test flaky.
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_artifacts")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"run_{os.getpid()}.csv")
        try:
            recorder = CsvRecorder(path)
            app = App(
                SimulatedTransport(hz=100.0, period_s=1.0),
                cfg=AppConfig(control_hz=50.0, stats_interval_s=1e9, print_data=False),
                recorder=recorder,
            )
            asyncio.run(_run_for(0.6, app))

            self.assertTrue(os.path.exists(path))
            with open(path, encoding="utf-8-sig") as fh:
                lines = fh.read().splitlines()
            self.assertGreater(len(lines), 20, "expected a header plus ~60 rows")
            self.assertIn("roll_deg", lines[0])
            self.assertEqual(len(lines[1].split(",")), len(lines[0].split(",")))
        finally:
            if os.path.exists(path):
                os.remove(path)

    def test_parser_stats_are_exposed(self) -> None:
        app = App(
            SimulatedTransport(hz=50.0, period_s=1.0),
            cfg=AppConfig(stats_interval_s=1e9, print_data=False),
        )
        asyncio.run(_run_for(0.4, app))
        self.assertGreater(app._parser.stats.frames_ok, 5)


if __name__ == "__main__":
    unittest.main()
