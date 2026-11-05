"""BLE transport built on ``bleak``.

``bleak`` is imported lazily so the rest of the package (parser, tests,
simulator, log analysis) works on a machine without Bluetooth.

Windows notes
-------------
* Connecting with the *scanned* ``BLEDevice`` object is much faster than with a
  bare address string (measured on a paired WT901BLE67: 4.8 s vs 11.2 s),
  because WinRT can skip re-resolution.  Both paths are kept: the object is
  tried first and the address is used as a fallback on reconnect.
* Only one process can hold the notification subscription.  The vendor app on a
  phone, or a previously crashed script, will silently block the connection.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Sequence

from .ble import COMMON_NOTIFY_HINTS, BleProfile, DiscoveredCharacteristic


@dataclass
class DeviceInfo:
    name: str
    address: str
    rssi: Optional[int] = None
    #: Opaque platform device object (``bleak.BLEDevice``); used to speed up
    #: connection and never compared/serialised.
    native: Any = field(default=None, repr=False, compare=False)

    def __str__(self) -> str:
        rssi = "-" if self.rssi is None else str(self.rssi)
        return f"{self.name or '<no-name>':32s} {self.address}  rssi={rssi}"


class TransportError(RuntimeError):
    pass


async def scan(timeout: float = 8.0) -> List[DeviceInfo]:
    from bleak import BleakScanner  # lazy import

    found = await BleakScanner.discover(timeout=timeout)
    out: List[DeviceInfo] = []
    for d in found:
        out.append(
            DeviceInfo(
                getattr(d, "name", "") or "",
                d.address,
                getattr(d, "rssi", None),
                native=d,
            )
        )
    out.sort(key=lambda x: x.name.lower())
    return out


async def find_device(
    *,
    name_contains: Optional[str] = None,
    address: Optional[str] = None,
    timeout: float = 8.0,
) -> Optional[DeviceInfo]:
    """Resolve a device, preferring a *filtered* scan.

    A filtered scan returns as soon as the target advertises (~2.5 s measured),
    and gives us the platform device object, which connects much faster than a
    bare address (~5 s vs ~11 s on a paired WT901BLE67).
    """

    from bleak import BleakScanner  # lazy import

    if address:
        try:
            dev = await BleakScanner.find_device_by_filter(
                lambda d, _ad: d.address.lower() == address.lower(), timeout=timeout
            )
        except Exception:
            dev = None
        if dev is not None:
            return DeviceInfo(dev.name or "", dev.address, getattr(dev, "rssi", None), native=dev)
        return DeviceInfo("", address)

    if not name_contains:
        raise TransportError("either address or name_contains is required")
    needle = name_contains.lower()

    def matches(device, advertisement) -> bool:
        name = (getattr(device, "name", "") or getattr(advertisement, "local_name", "") or "")
        return needle in name.lower()

    try:
        dev = await BleakScanner.find_device_by_filter(matches, timeout=timeout)
    except Exception:
        dev = None
    if dev is None:
        return None
    return DeviceInfo(dev.name or "", dev.address, getattr(dev, "rssi", None), native=dev)


# ---------------------------------------------------------------- selection
def pick_notify_uuid(
    characteristics: Sequence[DiscoveredCharacteristic],
    *,
    profile: Optional[BleProfile] = None,
    explicit: Optional[str] = None,
    hints: Sequence[str] = COMMON_NOTIFY_HINTS,
) -> str:
    """Choose the data characteristic.

    Priority: explicit override -> profile -> single candidate -> short-UUID
    hints.  Pure function so the rule can be unit tested without hardware.
    """

    candidates = [
        c for c in characteristics if c.has("notify") or c.has("indicate")
    ]

    if explicit:
        return explicit

    if profile is not None and profile.notify_uuid:
        wanted = profile.notify_uuid.lower()
        for c in candidates:
            if c.uuid.lower() == wanted:
                return c.uuid

    if not candidates:
        raise TransportError(
            "no notify characteristic found - is this really the IMU? "
            "check that the vendor app is disconnected"
        )
    if len(candidates) == 1:
        return candidates[0].uuid

    lowered = [(c.uuid.lower(), c) for c in candidates]
    for hint in hints:
        hits = [c for uuid, c in lowered if hint in uuid]
        if len(hits) == 1:
            return hits[0].uuid

    listing = "\n  ".join(f"{c.uuid} {list(c.properties)}" for c in candidates)
    raise TransportError(
        "several notify characteristics, cannot choose automatically; "
        f"pass notify_uuid=:\n  {listing}"
    )


def pick_write_uuid(
    characteristics: Sequence[DiscoveredCharacteristic],
    *,
    profile: Optional[BleProfile] = None,
) -> Optional[str]:
    """Find a writable characteristic (needed only for register commands)."""

    if profile is not None and profile.write_uuid:
        wanted = profile.write_uuid.lower()
        for c in characteristics:
            if c.uuid.lower() == wanted:
                return c.uuid

    for c in characteristics:
        if c.has("write") or c.has("write-without-response"):
            return c.uuid
    return None


class BleTransport:
    """Connects, subscribes to the notify characteristic and pumps bytes."""

    def __init__(
        self,
        *,
        address: Optional[str] = None,
        name_contains: Optional[str] = None,
        notify_uuid: Optional[str] = None,
        write_uuid: Optional[str] = None,
        profile: Optional[BleProfile] = None,
        connect_timeout: float = 20.0,
        scan_timeout: float = 8.0,
        verbose: bool = True,
    ) -> None:
        if not address and not name_contains:
            raise ValueError("need address= or name_contains=")
        self._address = address
        self._name_contains = name_contains
        self._notify_uuid = notify_uuid
        self._write_uuid = write_uuid
        self._profile = profile
        self._connect_timeout = connect_timeout
        self._scan_timeout = scan_timeout
        self._verbose = verbose

        self.notify_uuid: Optional[str] = None
        self.write_uuid: Optional[str] = None
        self.device: Optional[DeviceInfo] = None
        self._client: Any = None

    # ------------------------------------------------------------------ run
    async def run(self, on_bytes: Callable[[bytes], None]) -> None:
        from bleak import BleakClient  # lazy import

        if self.device is None:
            dev = await find_device(
                name_contains=self._name_contains,
                address=self._address,
                timeout=self._scan_timeout,
            )
            if dev is None:
                raise TransportError(
                    f"找不到设备 {self._name_contains!r}：可能未上电、超出范围，"
                    "或正被维特上位机/手机 App 连接（BLE 被占用时设备停止广播）。"
                    "关闭占用程序后本程序会自动重连，无需重启。"
                )
            self.device = dev

        self._log(f"[ble] connecting to {self.device.address} ({self.device.name or '?'})")
        client = await self._connect_with_fallback(BleakClient)
        self._client = client

        try:
            self._log("[ble] connected, discovering services")
            characteristics = [
                DiscoveredCharacteristic(c.uuid, tuple(c.properties))
                for service in client.services
                for c in service.characteristics
            ]
            for c in characteristics:
                self._log(f"[ble]   char {c.uuid} {list(c.properties)}")

            uuid = pick_notify_uuid(
                characteristics, profile=self._profile, explicit=self._notify_uuid
            )
            self.notify_uuid = uuid
            self._log(f"[ble] subscribe {uuid}")
            await client.start_notify(uuid, lambda _sender, data: on_bytes(bytes(data)))

            self.write_uuid = self._write_uuid or pick_write_uuid(
                characteristics, profile=self._profile
            )

            start = self._profile.start_command if self._profile else None
            if start and self.write_uuid:
                self._log(f"[ble] send start command {start.hex(' ')}")
                await client.write_gatt_char(self.write_uuid, start, response=False)

            self._log("[ble] notification stream started")

            while client.is_connected:
                await asyncio.sleep(0.2)
            raise ConnectionError("BLE link dropped")
        finally:
            try:
                if client.is_connected and self.notify_uuid:
                    await client.stop_notify(self.notify_uuid)
            except Exception:
                pass
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception:
                pass
            self._client = None

    async def _connect_with_fallback(self, client_cls) -> Any:
        """Fast path first; if it fails, rescan by name and retry once.

        Direct connect by a remembered address saves the scan (~6-8 s), while
        the fallback keeps working when the device address changed or the
        cached advertisement is stale.
        """

        try:
            return await self._connect(client_cls)
        except Exception as exc:  # noqa: BLE001 - any failure worth one retry
            self._log(f"[ble] direct connect failed ({exc}); rescanning")
            if not self._name_contains:
                raise TransportError(
                    f"连接 {self._address} 失败：设备可能未上电、超出范围，"
                    "或正被其他程序连接。"
                ) from exc
            dev = await find_device(
                name_contains=self._name_contains, timeout=self._scan_timeout
            )
            if dev is None:
                raise TransportError(
                    f"找不到设备 {self._name_contains!r}：可能未上电、超出范围，"
                    "或正被维特上位机/手机 App 连接（BLE 被占用时设备停止广播）。"
                    "关闭占用程序后本程序会自动重连，无需重启。"
                ) from exc
            self.device = dev
            return await self._connect(client_cls)

    async def _connect(self, client_cls) -> Any:
        """Prefer the scanned device object; fall back to the address string."""

        target = self.device.native if self.device and self.device.native is not None else None
        if target is not None:
            try:
                client = client_cls(target, timeout=self._connect_timeout)
                await client.connect()
                return client
            except Exception as exc:  # stale object after sleep/re-pair
                self._log(f"[ble] connect via device object failed ({exc}); retrying by address")

        assert self.device is not None
        client = client_cls(self.device.address, timeout=self._connect_timeout)
        await client.connect()
        return client

    # --------------------------------------------------------------- commands
    async def send(self, data: bytes, *, response: bool = False) -> None:
        """Write a register command to the device (FF AA frames, see ``ble``)."""

        if self._client is None or not self._client.is_connected:
            raise TransportError("not connected")
        if not self.write_uuid:
            raise TransportError("device has no writable characteristic")
        await self._client.write_gatt_char(self.write_uuid, data, response=response)

    async def close(self) -> None:
        client = self._client
        self._client = None
        if client is None:
            return
        try:
            if client.is_connected:
                await client.disconnect()
        except Exception:
            pass

    def _log(self, message: str) -> None:
        if self._verbose:
            print(message, flush=True)
