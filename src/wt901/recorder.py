"""CSV recorder for raw decoded samples."""

from __future__ import annotations

import csv
import os
import time
from typing import Optional, TextIO

from .controller import ControlCommand
from .model import ImuSample

COLUMNS = [
    "t_wall_iso",
    "t_mono",
    "seq",
    "kind",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "mx",
    "my",
    "mz",
    "temp_c",
    "cmd",
    "cmd_value",
]


def _iso(ts: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts)) + f".{int((ts % 1) * 1000):03d}"


def _num(value: Optional[float]) -> str:
    return "" if value is None else f"{value:.4f}"


class CsvRecorder:
    """Buffered CSV writer; flushes on a timer so a crash loses little data."""

    def __init__(self, path: str, *, flush_interval: float = 1.0) -> None:
        self._path = path
        self._flush_interval = flush_interval
        self._fh: Optional[TextIO] = None
        self._writer = None
        self._last_flush = 0.0
        self.rows = 0

    def open(self) -> None:
        directory = os.path.dirname(os.path.abspath(self._path))
        os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(self._path) or os.path.getsize(self._path) == 0
        # utf-8-sig keeps Excel happy on Chinese Windows.
        self._fh = open(self._path, "a", encoding="utf-8-sig", newline="")
        self._writer = csv.writer(self._fh)
        if new_file:
            self._writer.writerow(COLUMNS)
        self._last_flush = time.monotonic()

    def write(self, sample: ImuSample, cmd: Optional[ControlCommand] = None) -> None:
        if self._writer is None:
            return
        self._writer.writerow(
            [
                _iso(sample.t_wall or time.time()),
                f"{sample.t_mono:.6f}",
                sample.seq,
                sample.kind,
                _num(sample.roll),
                _num(sample.pitch),
                _num(sample.yaw),
                _num(sample.ax),
                _num(sample.ay),
                _num(sample.az),
                _num(sample.gx),
                _num(sample.gy),
                _num(sample.gz),
                _num(sample.mx),
                _num(sample.my),
                _num(sample.mz),
                _num(sample.temp_c),
                cmd.name if cmd else "",
                f"{cmd.value:.3f}" if cmd else "",
            ]
        )
        self.rows += 1
        now = time.monotonic()
        if now - self._last_flush >= self._flush_interval:
            self._fh.flush()  # type: ignore[union-attr]
            self._last_flush = now

    def close(self) -> None:
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            self._writer = None
