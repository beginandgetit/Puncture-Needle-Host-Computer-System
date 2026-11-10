"""WT9011DCL-BT50 (WitMotion BLE IMU) SDK.

Quick start::

    from wt901 import WT901, WT901Config

    cfg = WT901Config(address="AA:BB:CC:DD:EE:FF")
    with WT901(cfg) as imu:
        reading = imu.read(timeout=2.0)
        print(reading.pitch, reading.ax, reading.gy)

Lower-level building blocks (parser, controller, sinks) are exported as well,
so a product can reuse the decoding without the connection management.
"""

__version__ = "0.6.0"

from .ble import WITMOTION_BLE, BleProfile, wit_frame, wit_save, wit_unlock
from .controller import (
    FWD,
    REV,
    STOP,
    AttitudeController,
    ControlCommand,
    ControlConfig,
)
from .devices import load_last_device, save_last_device
from .errors import (
    CalibrationError,
    ConfigError,
    ConnectionTimeout,
    LinkError,
    NoDataError,
    NotStartedError,
    WT901Error,
)
from .estimator import NeedlePose, NeedleTracker
from .mapping import (
    canonical_placement,
    model_rotation,
    quaternion_from_reading,
    reading_quaternion,
)
from .model import ImuSample
from .probe import CaptureAnalysis, ProbeReport, analyze_capture
from .protocol import Scales, WitParser, encode_combined, encode_quaternion, encode_split
from .sensor import AsyncWT901, ImuReading, LinkState, SensorStatus, WT901, WT901Config

__all__ = [
    # high level
    "WT901",
    "AsyncWT901",
    "WT901Config",
    "ImuReading",
    "SensorStatus",
    "LinkState",
    # device / BLE
    "BleProfile",
    "WITMOTION_BLE",
    "wit_frame",
    "wit_save",
    "wit_unlock",
    "load_last_device",
    "save_last_device",
    # needle angle
    "NeedleTracker",
    "NeedlePose",
    # sensor <-> needle model mapping
    "canonical_placement",
    "model_rotation",
    "quaternion_from_reading",
    "reading_quaternion",
    # diagnostics
    "analyze_capture",
    "CaptureAnalysis",
    "ProbeReport",
    # control helpers
    "AttitudeController",
    "ControlConfig",
    "ControlCommand",
    "FWD",
    "REV",
    "STOP",
    # low level
    "WitParser",
    "Scales",
    "ImuSample",
    "encode_combined",
    "encode_quaternion",
    "encode_split",
    # errors
    "WT901Error",
    "ConfigError",
    "LinkError",
    "ConnectionTimeout",
    "NotStartedError",
    "NoDataError",
    "CalibrationError",
    "__version__",
]
