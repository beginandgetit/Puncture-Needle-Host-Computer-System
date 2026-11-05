"""`wt901 probe` - find out what the device actually sends.

The three unknowns that block a real integration are: which notify
characteristic carries data, which frame layout is used and at what rate.  The
probe answers all three from the live device and prints a report that can be
pasted back into a bug report.

The pure analysis half (:func:`analyze_capture`) has no BLE dependency, so it
is unit tested and can also be run on a saved capture:

    python -m wt901 analyze --file capture.bin
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .protocol import (
    ID_ACCEL,
    ID_ANGLE,
    ID_COMBINED,
    ID_GYRO,
    ID_MAG,
    Scales,
    WitParser,
)

_KIND_NAMES = {
    ID_ACCEL: "0x51 accel",
    ID_GYRO: "0x52 gyro",
    ID_ANGLE: "0x53 angle",
    ID_MAG: "0x54 mag",
    ID_COMBINED: "0x61 combined",
}


@dataclass
class CaptureAnalysis:
    total_bytes: int
    frames_ok: int
    frames_by_id: Dict[int, int]
    bad_checksum: int
    bad_header: int
    format_name: str
    duration_s: Optional[float]
    rate_hz: Optional[float]
    accel_norm: Optional[float]
    samples: List[ImuSample] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    alternative: Optional[str] = None

    def as_text(self) -> str:
        lines = [
            f"bytes={self.total_bytes} frames={self.frames_ok} "
            f"bad_sum={self.bad_checksum} bad_hdr={self.bad_header}",
            f"format: {self.format_name}",
        ]
        if self.frames_by_id:
            ids = ", ".join(f"0x{k:02X}={v}" for k, v in sorted(self.frames_by_id.items()))
            lines.append(f"frames by id: {ids}")
        if self.rate_hz is not None:
            lines.append(f"rate: {self.rate_hz:.1f} Hz (over {self.duration_s:.1f}s)")
        if self.accel_norm is not None:
            lines.append(f"|accel| = {self.accel_norm:.3f} g (should be ~1.000 at rest)")
        if self.alternative:
            lines.append(f"alternative: {self.alternative}")
        if self.samples:
            lines.append("first samples:")
            for s in self.samples[:5]:
                lines.append(
                    f"  {s.kind:9s} r={_f(s.roll):>8} p={_f(s.pitch):>8} y={_f(s.yaw):>8} "
                    f"a=({_f(s.ax)},{_f(s.ay)},{_f(s.az)}) g=({_f(s.gx)},{_f(s.gy)},{_f(s.gz)})"
                )
        for note in self.notes:
            lines.append(f"! {note}")
        return "\n".join(lines)


def _f(value: Optional[float], width: int = 8) -> str:
    return "n/a".rjust(width) if value is None else f"{value:.2f}".rjust(width)


def analyze_capture(
    data: bytes,
    *,
    duration_s: Optional[float] = None,
    scales: Scales = Scales(),
    limit: int = 5,
) -> CaptureAnalysis:
    """Decode a raw byte capture and describe what the device is doing."""

    parser = WitParser(scales=scales)
    samples = parser.feed(data)
    ids = dict(parser.stats.frames_by_id)

    format_name = "unknown"
    notes: List[str] = []
    alternative: Optional[str] = None

    if samples:
        if set(ids) == {ID_COMBINED}:
            format_name = "0x55 0x61 combined, 20 bytes, no checksum"
        elif set(ids) <= {ID_ACCEL, ID_GYRO, ID_ANGLE, ID_MAG}:
            format_name = "0x55 0x51/0x52/0x53 split frames, 11 bytes, checksummed"
        else:
            format_name = "mixed"
            notes.append("several frame types decoded; check the device output configuration")
    else:
        notes.append(
            "no frame decoded: layout may differ, bytes may be encrypted/compressed, "
            "or the characteristic is not the data channel"
        )
        alt = WitParser(scales=scales, combined_checksum=True)
        alt_samples = alt.feed(data)
        if alt_samples:
            alternative = (
                f"21-byte combined frame with trailing checksum decoded "
                f"({len(alt_samples)} frames) - pass --combined-checksum"
            )

    rate = None
    if duration_s and duration_s > 0 and samples:
        rate = len(samples) / duration_s

    accel_norm = None
    for s in samples:
        if s.ax is not None and s.ay is not None and s.az is not None:
            accel_norm = (s.ax * s.ax + s.ay * s.ay + s.az * s.az) ** 0.5
            break
    if accel_norm is not None and not (0.85 <= accel_norm <= 1.15):
        notes.append(
            f"|accel|={accel_norm:.2f} g is far from 1 g: wrong range scale, or the "
            "sensor was moving during the capture"
        )

    if parser.stats.bad_checksum > max(5, parser.stats.frames_ok // 20):
        notes.append(
            "many checksum failures: frame length or checksum position is wrong for this firmware"
        )

    if rate is not None and rate < 10:
        notes.append(f"only {rate:.1f} Hz: device ODR or BLE connection interval is low")

    return CaptureAnalysis(
        total_bytes=len(data),
        frames_ok=len(samples),
        frames_by_id=ids,
        bad_checksum=parser.stats.bad_checksum,
        bad_header=parser.stats.bad_header,
        format_name=format_name,
        duration_s=duration_s,
        rate_hz=rate,
        accel_norm=accel_norm,
        samples=samples[:limit],
        notes=notes,
        alternative=alternative,
    )


# ------------------------------------------------------------------- live probe
@dataclass
class CharacteristicInfo:
    uuid: str
    properties: Tuple[str, ...]
    bytes_received: int = 0

    def as_text(self) -> str:
        return f"{self.uuid} {list(self.properties)} bytes={self.bytes_received}"


@dataclass
class ProbeReport:
    device_name: str
    address: str
    services: List[str]
    characteristics: List[CharacteristicInfo]
    chosen_uuid: Optional[str]
    analysis: Optional[CaptureAnalysis]
    duration_s: float
    capture: bytes = b""
    warnings: List[str] = field(default_factory=list)

    def as_text(self) -> str:
        lines = [
            "=" * 72,
            "WT901 probe report",
            "=" * 72,
            f"device : {self.device_name or '<no-name>'}  {self.address}",
            f"capture: {self.duration_s:.1f} s, {len(self.capture)} bytes",
            "",
            "services:",
        ]
        lines.extend(f"  {s}" for s in self.services)
        lines.append("")
        lines.append("characteristics:")
        for c in self.characteristics:
            lines.append(f"  {c.as_text()}")
        lines.append("")
        if self.chosen_uuid:
            lines.append(f"selected data characteristic: {self.chosen_uuid}")
        if self.analysis:
            lines.append("")
            lines.append(self.analysis.as_text())
            lines.append("")
            lines.append("suggested SDK configuration:")
            lines.append(f"  notify_uuid={self.chosen_uuid!r}")
            if self.analysis.frames_ok and set(self.analysis.frames_by_id) == {ID_COMBINED}:
                lines.append("  # combined 0x61 frames -> defaults are fine")
            elif self.analysis.frames_ok:
                lines.append("  # split frames -> defaults are fine, frames are merged for you")
        for warning in self.warnings:
            lines.append(f"! {warning}")
        lines.append("=" * 72)
        return "\n".join(lines)


async def probe_device(
    *,
    address: Optional[str] = None,
    name_contains: str = "WT901",
    duration: float = 6.0,
    scan_timeout: float = 8.0,
) -> ProbeReport:
    """Connect, enumerate GATT, capture every notify characteristic, analyse.

    Subscribes to *all* notify characteristics at once so the data channel is
    identified by traffic volume instead of by guessing the UUID.
    """

    from bleak import BleakClient  # lazy: probe is the only part that needs it

    from .transport import DeviceInfo, scan

    devices = await scan(timeout=scan_timeout)
    if address:
        target = next((d for d in devices if d.address.lower() == address.lower()), None)
        if target is None:
            target = DeviceInfo("", address)
    else:
        matches = [d for d in devices if name_contains.lower() in (d.name or "").lower()]
        if not matches:
            raise RuntimeError(
                f"no device matching {name_contains!r}; found: "
                + (", ".join(f"{d.name}@{d.address}" for d in devices) or "nothing")
            )
        target = matches[0]

    if target is None:
        raise RuntimeError("no BLE device found")

    client = BleakClient(target.address, timeout=15.0)
    await client.connect()
    try:
        services: List[str] = []
        chars: List[CharacteristicInfo] = []
        notify_uuids: List[str] = []
        for service in client.services:
            services.append(str(service.uuid))
            for char in service.characteristics:
                info = CharacteristicInfo(char.uuid, tuple(char.properties))
                chars.append(info)
                if "notify" in char.properties or "indicate" in char.properties:
                    notify_uuids.append(char.uuid)

        buffers: Dict[str, bytearray] = {uuid: bytearray() for uuid in notify_uuids}

        def make_handler(uuid: str):
            def handler(_sender, data: bytearray) -> None:
                buffers[uuid].extend(data)

            return handler

        subscribed: List[str] = []
        warnings: List[str] = []
        for uuid in notify_uuids:
            try:
                await client.start_notify(uuid, make_handler(uuid))
                subscribed.append(uuid)
            except Exception as exc:  # noqa: BLE001 - some chars refuse notify
                warnings.append(f"could not subscribe {uuid}: {exc}")

        if not subscribed:
            warnings.append("no notify characteristic accepted a subscription")

        await asyncio.sleep(duration)
        for uuid in subscribed:
            try:
                await client.stop_notify(uuid)
            except Exception:
                pass
    finally:
        try:
            if client.is_connected:
                await client.disconnect()
        except Exception:
            pass

    for info in chars:
        info.bytes_received = len(buffers.get(info.uuid, b""))

    chosen = max(buffers.items(), key=lambda kv: len(kv[1]), default=(None, bytearray()))
    chosen_uuid = chosen[0] if chosen[1] else None
    capture = bytes(chosen[1]) if chosen_uuid else b""

    analysis = analyze_capture(capture, duration_s=duration) if capture else None

    report = ProbeReport(
        device_name=target.name,
        address=target.address,
        services=services,
        characteristics=chars,
        chosen_uuid=chosen_uuid,
        analysis=analysis,
        duration_s=duration,
        capture=capture,
        warnings=warnings,
    )
    if not capture:
        report.warnings.append(
            "no bytes arrived on any notify characteristic. Close the vendor app "
            "(it holds the BLE connection), power-cycle the sensor, and retry. If it "
            "still stays silent the module may need an explicit start command."
        )
    return report


def report_to_dict(report: ProbeReport) -> dict:
    return {
        "device": {"name": report.device_name, "address": report.address},
        "services": report.services,
        "characteristics": [
            {"uuid": c.uuid, "properties": list(c.properties), "bytes": c.bytes_received}
            for c in report.characteristics
        ],
        "chosen_uuid": report.chosen_uuid,
        "duration_s": report.duration_s,
        "analysis": None
        if report.analysis is None
        else {
            "frames": report.analysis.frames_ok,
            "format": report.analysis.format_name,
            "rate_hz": report.analysis.rate_hz,
            "accel_norm": report.analysis.accel_norm,
            "by_id": {hex(k): v for k, v in report.analysis.frames_by_id.items()},
            "notes": report.analysis.notes,
        },
        "warnings": report.warnings,
    }
