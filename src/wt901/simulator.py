"""Hardware-free data source.

Used by ``wt901 sim`` and by the self-test so the whole pipeline (parser ->
buffer -> controller -> sinks -> recorder) can be exercised on a laptop with no
sensor, and in CI.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Callable, Optional

from .protocol import Scales, encode_combined, encode_split


class SimulatedTransport:
    """Emits synthetic attitude data at a fixed rate.

    The default profile tilts pitch between -20 and +20 degrees with a 2 s
    period, which crosses the default 15 deg threshold in both directions, so
    FWD -> STOP -> REV -> STOP transitions are all visible within ~4 s.
    """

    def __init__(
        self,
        *,
        hz: float = 60.0,
        pitch_amp: float = 20.0,
        period_s: float = 2.0,
        split: bool = False,
        noise_deg: float = 0.15,
        scales: Scales = Scales(),
        fail_after_s: Optional[float] = None,
        verbose: bool = False,
    ) -> None:
        if hz <= 0:
            raise ValueError("hz must be > 0")
        self._hz = hz
        self._amp = pitch_amp
        self._period = max(period_s, 1e-3)
        self._split = split
        self._noise = noise_deg
        self._scales = scales
        self._fail_after = fail_after_s
        self._verbose = verbose
        self.sent = 0

    async def run(self, on_bytes: Callable[[bytes], None]) -> None:
        dt = 1.0 / self._hz
        start = time.monotonic()
        n = 0
        while True:
            t = time.monotonic() - start
            if self._fail_after is not None and t >= self._fail_after:
                raise ConnectionError("simulated link loss")
            n += 1

            pitch = self._amp * math.sin(2.0 * math.pi * t / self._period)
            roll = 2.0 * math.sin(2.0 * math.pi * t / 5.0)
            yaw = ((t * 15.0 + 180.0) % 360.0) - 180.0

            # Crude derivative: enough to look plausible in the logs.
            d_pitch = self._amp * (2.0 * math.pi / self._period) * math.cos(
                2.0 * math.pi * t / self._period
            )
            d_roll = 2.0 * (2.0 * math.pi / 5.0) * math.cos(2.0 * math.pi * t / 5.0)

            jitter = self._noise * math.sin(n * 0.7)
            pitch += jitter
            roll += jitter * 0.5

            pr = math.radians(pitch)
            rr = math.radians(roll)
            ax = -math.sin(pr)
            ay = math.sin(rr) * math.cos(pr)
            az = math.cos(pr) * math.cos(rr)

            if self._split:
                on_bytes(encode_split(0x51, ax, ay, az, self._scales))
                on_bytes(encode_split(0x52, d_roll, d_pitch, 0.0, self._scales))
                on_bytes(encode_split(0x53, roll, pitch, yaw, self._scales))
            else:
                on_bytes(
                    encode_combined(
                        ax, ay, az, d_roll, d_pitch, 0.0, roll, pitch, yaw, self._scales
                    )
                )
            self.sent += 1

            await asyncio.sleep(dt)

    async def close(self) -> None:
        return None
