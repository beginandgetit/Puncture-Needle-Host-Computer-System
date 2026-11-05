"""Latest-value telemetry buffer.

The BLE link is a push source with jitter, duplicates and occasional gaps.  The
control loop must run on a fixed period instead of being driven by packet
arrival, so arrivals are merged into a "latest value per field" buffer and the
control loop only reads snapshots.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .model import ImuSample

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Snapshot:
    t_mono: float
    accel: Optional[Vec3]
    gyro: Optional[Vec3]
    attitude: Optional[Vec3]
    attitude_t: float
    accel_t: float
    gyro_t: float

    def age_attitude(self, now: float) -> float:
        return now - self.attitude_t

    def age_accel(self, now: float) -> float:
        return now - self.accel_t

    def age_gyro(self, now: float) -> float:
        return now - self.gyro_t

    @classmethod
    def from_reading(cls, reading, now: Optional[float] = None) -> "Snapshot":
        """Adapt an SDK ``ImuReading`` to the controller's input type.

        Duck-typed on purpose: this module must not import the SDK, otherwise
        the dependency direction (controller <- sensor) would invert.
        """

        t = reading.t_mono if now is None else now
        attitude = (
            (reading.roll, reading.pitch, reading.yaw)
            if getattr(reading, "has_attitude", True)
            else None
        )
        return cls(
            t_mono=t,
            accel=(reading.ax, reading.ay, reading.az),
            gyro=(reading.gx, reading.gy, reading.gz),
            attitude=attitude,
            attitude_t=reading.t_mono,
            accel_t=reading.t_mono,
            gyro_t=reading.t_mono,
        )


class TelemetryBuffer:
    """Merges partial frames (0x51/0x52/0x53) into one consistent view."""

    __slots__ = ("_accel", "_gyro", "_attitude", "_t_accel", "_t_gyro", "_t_att", "_last")

    def __init__(self) -> None:
        self._accel: Optional[Vec3] = None
        self._gyro: Optional[Vec3] = None
        self._attitude: Optional[Vec3] = None
        self._t_accel = 0.0
        self._t_gyro = 0.0
        self._t_att = 0.0
        self._last: Optional[ImuSample] = None

    @property
    def last_sample(self) -> Optional[ImuSample]:
        return self._last

    def update(self, sample: ImuSample) -> None:
        self._last = sample
        t = sample.t_mono
        if sample.has_attitude():
            self._attitude = (sample.roll, sample.pitch, sample.yaw)  # type: ignore[arg-type]
            self._t_att = t
        if sample.ax is not None and sample.ay is not None and sample.az is not None:
            self._accel = (sample.ax, sample.ay, sample.az)
            self._t_accel = t
        if sample.gx is not None and sample.gy is not None and sample.gz is not None:
            self._gyro = (sample.gx, sample.gy, sample.gz)
            self._t_gyro = t

    def snapshot(self, now: float) -> Snapshot:
        return Snapshot(
            t_mono=now,
            accel=self._accel,
            gyro=self._gyro,
            attitude=self._attitude,
            attitude_t=self._t_att,
            accel_t=self._t_accel,
            gyro_t=self._t_gyro,
        )

    def reset(self) -> None:
        """Drop all values, e.g. after a disconnect so nothing stale is reused."""

        self._accel = None
        self._gyro = None
        self._attitude = None
        self._t_accel = 0.0
        self._t_gyro = 0.0
        self._t_att = 0.0


def norm(v: Optional[Vec3]) -> Optional[float]:
    if v is None:
        return None
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
