"""Exception hierarchy for the public SDK.

Everything raised by the SDK derives from :class:`WT901Error`, so a product can
wrap a single ``except`` around acquisition code if it does not care about the
specific failure.
"""

from __future__ import annotations


class WT901Error(Exception):
    """Base class for every SDK error."""


class ConfigError(WT901Error, ValueError):
    """Invalid configuration (bad axis, missing address, ...)."""


class LinkError(WT901Error):
    """The BLE link could not be established or was lost."""


class ConnectionTimeout(LinkError, TimeoutError):
    """Connected was requested but no data arrived in time."""


class NotStartedError(WT901Error):
    """The sensor has not been started."""


class NoDataError(WT901Error, TimeoutError):
    """``read()`` timed out before a new reading arrived."""


class CalibrationError(WT901Error):
    """Calibration could not be completed (usually: not enough samples)."""
