"""BLE profiles and WitMotion register commands.

A profile pins the GATT layout of a device family so the transport never has to
guess UUIDs.  The defaults below were read from a real **WT901BLE67** with
``wt901 probe``:

    service  0000ffe5-0000-1000-8000-00805f9a34fb
    notify   0000ffe4-0000-1000-8000-00805f9a34fb   (sensor data)
    write    0000ffe9-0000-1000-8000-00805f9a34fb   (FF AA register writes)

Note the base UUID ends with ``9a34fb`` (WitMotion's variant), not the standard
``9b34fb``.  Matching is done on the 16-bit short value, so both work.

Register commands use the standard WitMotion frame::

    FF AA <reg> <data_l> <data_h> <sum>     sum = (0xFF+0xAA+reg+datal+datah) & 0xFF

They are provided for the next feature (changing output rate / content); the
prototype does **not** write to the device automatically.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class BleProfile:
    """GATT layout + optional wake-up command for one device family."""

    name: str
    service_uuid: Optional[str] = None
    notify_uuid: Optional[str] = None
    write_uuid: Optional[str] = None
    start_command: Optional[bytes] = None
    notes: str = ""

    def short(self, uuid: str) -> str:
        """16-bit short form of a UUID, for logs."""

        u = uuid.lower()
        if u.startswith("0000") and len(u) >= 8:
            return u[4:8]
        return u


# Verified on WT901BLE67 / 2026-11-05 with `wt901 probe`.
WITMOTION_BLE = BleProfile(
    name="WitMotion BLE (FFE5/FFE4/FFE9)",
    service_uuid="0000ffe5-0000-1000-8000-00805f9a34fb",
    notify_uuid="0000ffe4-0000-1000-8000-00805f9a34fb",
    write_uuid="0000ffe9-0000-1000-8000-00805f9a34fb",
    start_command=None,  # this module streams right after subscribing
    notes="WT901BLE67: 0x55 0x61 combined frames, 20 bytes, 10 Hz out of the box",
)

# Other WitMotion modules commonly expose FFE1 (HM-10 style); kept as a hint
# list for devices without a pinned profile.
COMMON_NOTIFY_HINTS: Tuple[str, ...] = ("ffe4", "ffe1", "ffe9", "6e400003", "fff1", "fff2")


def wit_frame(register: int, value: int = 0) -> bytes:
    """Build a WitMotion ``FF AA`` register write frame."""

    if not 0 <= register <= 0xFF:
        raise ValueError("register must fit in one byte")
    if not 0 <= value <= 0xFFFF:
        raise ValueError("value must fit in two bytes")
    body = bytes([0xFF, 0xAA, register, value & 0xFF, (value >> 8) & 0xFF])
    return body + bytes([sum(body) & 0xFF])


def wit_save() -> bytes:
    """Persist the current register settings."""

    return wit_frame(0x00, 0x0000)


def wit_unlock() -> bytes:
    """Wake-up command used by some WitMotion BLE modules (not needed here)."""

    return bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])


@dataclass
class DiscoveredCharacteristic:
    """Minimal, transport-independent view of a GATT characteristic."""

    uuid: str
    properties: Sequence[str] = field(default_factory=tuple)

    def has(self, prop: str) -> bool:
        return prop in self.properties
