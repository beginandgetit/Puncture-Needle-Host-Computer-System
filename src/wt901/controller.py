"""Attitude -> command state machine.

Design rules (these matter more than the exact numbers):

1. *Fail safe*: no fresh attitude data means STOP, never "keep the last
   command".  A stale, missing or implausible attitude is treated identically.
2. *Hysteresis*: a threshold crossing alone is not enough to leave a state;
   the value must come back past ``threshold - hysteresis``.  This is what
   keeps a sensor parked near 15 deg from chattering.
3. *Slew limit*: the command ramps towards its target instead of jumping, so a
   downstream actuator never receives a step demand.
4. *Deterministic*: the controller is a pure function of (snapshot, now) plus
   its internal state; time is always injected, so it is unit testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .state import Snapshot

FWD = "FWD"
REV = "REV"
STOP = "STOP"


@dataclass(frozen=True)
class ControlCommand:
    name: str
    value: float          # normalised, -1.0 .. +1.0
    reason: str
    t_mono: float
    valid: bool = True    # False => signal lost / rejected, treat as fail-safe

    def as_dict(self) -> dict:
        return {
            "cmd": self.name,
            "value": round(self.value, 4),
            "reason": self.reason,
            "valid": self.valid,
            "t": round(self.t_mono, 6),
        }


@dataclass
class ControlConfig:
    axis: str = "pitch"                # roll | pitch | yaw
    threshold_deg: float = 15.0
    hysteresis_deg: float = 3.0
    stale_timeout_s: float = 0.5       # no attitude frame -> STOP
    tilt_limit_deg: float = 60.0       # absolute safety cut
    slew_per_s: float = 2.0            # normalised units per second
    accel_gate_g: Optional[float] = None  # e.g. 0.5 => |a| must be within 1+/-0.5 g

    def validate(self) -> None:
        if self.axis not in ("roll", "pitch", "yaw"):
            raise ValueError(f"axis must be roll/pitch/yaw, got {self.axis!r}")
        if self.threshold_deg <= 0:
            raise ValueError("threshold_deg must be > 0")
        if not 0 <= self.hysteresis_deg < self.threshold_deg:
            raise ValueError("0 <= hysteresis_deg < threshold_deg required")
        if self.stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be > 0")


class AttitudeController:
    def __init__(self, cfg: ControlConfig = ControlConfig()) -> None:
        cfg.validate()
        self.cfg = cfg
        self._dir = 0                 # -1 / 0 / +1 latched direction
        self._value = 0.0             # slew-limited output
        self._last_t: Optional[float] = None
        self._hold_reason = "init"

    # ------------------------------------------------------------------ step
    def step(self, snap: Snapshot, now: float) -> ControlCommand:
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        fail = self._check_valid(snap, now)
        if fail is not None:
            return self._stop(now, fail, valid=False)

        assert snap.attitude is not None
        roll, pitch, yaw = snap.attitude
        value = {"roll": roll, "pitch": pitch, "yaw": yaw}[self.cfg.axis]

        target = self._target(value)

        max_step = self.cfg.slew_per_s * dt
        if self._value < target:
            self._value = min(target, self._value + max_step)
        elif self._value > target:
            self._value = max(target, self._value - max_step)
        if abs(self._value) < 1e-9:
            self._value = 0.0

        if self._value > 0:
            name = FWD
        elif self._value < 0:
            name = REV
        else:
            name = STOP
        return ControlCommand(name, self._value, self._hold_reason, now, valid=True)

    def reset(self) -> None:
        self._dir = 0
        self._value = 0.0
        self._last_t = None
        self._hold_reason = "reset"

    # --------------------------------------------------------------- internal
    def _check_valid(self, snap: Snapshot, now: float) -> Optional[str]:
        if snap.attitude is None:
            return "no_attitude"
        age = snap.age_attitude(now)
        if age > self.cfg.stale_timeout_s:
            return f"stale:{age * 1000:.0f}ms"
        roll, pitch, yaw = snap.attitude
        if abs(roll) > self.cfg.tilt_limit_deg or abs(pitch) > self.cfg.tilt_limit_deg:
            return f"tilt_limit:r={roll:.1f},p={pitch:.1f}"
        if self.cfg.accel_gate_g is not None and snap.accel is not None:
            ax, ay, az = snap.accel
            magnitude = (ax * ax + ay * ay + az * az) ** 0.5
            if abs(magnitude - 1.0) > self.cfg.accel_gate_g:
                return f"accel_gate:{magnitude:.2f}g"
        return None

    def _target(self, value: float) -> float:
        t = self.cfg.threshold_deg
        h = self.cfg.hysteresis_deg

        if self._dir > 0:
            if value < t - h:
                self._dir = 0
                self._hold_reason = "hysteresis_release"
            else:
                self._hold_reason = "hold_fwd"
        elif self._dir < 0:
            if value > -t + h:
                self._dir = 0
                self._hold_reason = "hysteresis_release"
            else:
                self._hold_reason = "hold_rev"
        else:
            if value >= t:
                self._dir = 1
                self._hold_reason = "threshold_fwd"
            elif value <= -t:
                self._dir = -1
                self._hold_reason = "threshold_rev"
            else:
                self._hold_reason = "below_threshold"

        return float(self._dir)

    def _stop(self, now: float, reason: str, *, valid: bool) -> ControlCommand:
        self._dir = 0
        self._value = 0.0
        self._hold_reason = reason
        return ControlCommand(STOP, 0.0, reason, now, valid=valid)
