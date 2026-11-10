"""Data model shared by the parser, state buffer and recorder."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Logical origins of a decoded sample.  A single sample may carry only part of
# the information depending on which WitMotion frame it came from.
KIND_COMBINED = "combined"  # 0x55 0x61, accel + gyro + attitude in one frame
KIND_ACCEL = "accel"        # 0x55 0x51
KIND_GYRO = "gyro"          # 0x55 0x52
KIND_ANGLE = "angle"        # 0x55 0x53
KIND_MAG = "mag"            # 0x55 0x54
KIND_QUATERNION = "quaternion"  # 0x55 0x59, attitude as a quaternion

# Physical units used throughout the project (deliberately explicit):
#   acceleration : g
#   angular rate : deg/s
#   attitude     : deg
#   quaternion   : unitless, Hamilton (w, x, y, z)
#   magnetic     : raw LSB (conversion depends on the chip datasheet)
UNITS = {
    "ax": "g", "ay": "g", "az": "g",
    "gx": "deg/s", "gy": "deg/s", "gz": "deg/s",
    "roll": "deg", "pitch": "deg", "yaw": "deg",
    "qw": "-", "qx": "-", "qy": "-", "qz": "-",
    "mx": "lsb", "my": "lsb", "mz": "lsb",
    "temp_c": "degC",
}


@dataclass(slots=True)
class ImuSample:
    """One decoded frame.

    ``None`` means "not carried by this frame", which is different from 0.0.
    """

    kind: str
    t_mono: float                 # time.monotonic() at decode time
    t_wall: float = 0.0           # time.time() at decode time
    seq: int = 0                  # parser-assigned monotonic frame counter
    ax: Optional[float] = None
    ay: Optional[float] = None
    az: Optional[float] = None
    gx: Optional[float] = None
    gy: Optional[float] = None
    gz: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    # Hamilton quaternion (w, x, y, z) in the device's sensor frame.  Carried by
    # the 0x59 frame; ``None`` on devices/frames that only output Euler angles.
    qw: Optional[float] = None
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    mx: Optional[float] = None
    my: Optional[float] = None
    mz: Optional[float] = None
    temp_c: Optional[float] = None
    raw: bytes = b""

    def has_attitude(self) -> bool:
        return self.roll is not None and self.pitch is not None and self.yaw is not None

    def has_quaternion(self) -> bool:
        return (
            self.qw is not None
            and self.qx is not None
            and self.qy is not None
            and self.qz is not None
        )

    def as_dict(self) -> dict:
        return {
            "kind": self.kind,
            "seq": self.seq,
            "t_mono": self.t_mono,
            "t_wall": self.t_wall,
            "accel": [self.ax, self.ay, self.az],
            "gyro": [self.gx, self.gy, self.gz],
            "attitude": [self.roll, self.pitch, self.yaw],
            "quaternion": [self.qw, self.qx, self.qy, self.qz],
            "mag": [self.mx, self.my, self.mz],
            "temp_c": self.temp_c,
        }
