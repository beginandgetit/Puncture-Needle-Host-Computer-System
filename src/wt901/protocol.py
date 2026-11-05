"""WitMotion frame decoding.

Two encodings are supported because WitMotion ships both depending on firmware
and on how the device is configured:

* split frames, 11 bytes, checksum in the last byte::

      55 51 AxL AxH AyL AyH AzL AzH TL TH SUM      (0x51 acceleration + temp)
      55 52 WxL ...               TL TH SUM        (0x52 angular rate + temp)
      55 53 RollL ...             VL VH SUM        (0x53 attitude + version)
      55 54 HxL  ...              TL TH SUM        (0x54 magnetic field + temp)
      SUM = (sum of the first 10 bytes) & 0xFF

    Bytes 8..9 are int16/100 degC for 0x51/0x52/0x54 and a version word for
    0x53, which is why every split frame is 11 bytes and not 9.

* combined frame, 20 bytes, no checksum byte::

      55 61 + 9 x int16 little endian
      ax ay az gx gy gz roll pitch yaw

    (18 payload bytes + 2 header bytes = 20, so there is no room for a
    checksum; that is why the combined frame can only be sanity-checked.)

Everything here is pure Python and hardware independent, so it can be unit
tested and can also be used to re-parse a captured hex log.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .model import (
    ImuSample,
    KIND_ACCEL,
    KIND_ANGLE,
    KIND_COMBINED,
    KIND_GYRO,
    KIND_MAG,
)

HEADER = 0x55

ID_ACCEL = 0x51
ID_GYRO = 0x52
ID_ANGLE = 0x53
ID_MAG = 0x54
ID_COMBINED = 0x61

SPLIT_LEN = 11
COMBINED_LEN = 20

_FULL_SCALE = 32768.0

_ID_TO_KIND = {
    ID_ACCEL: KIND_ACCEL,
    ID_GYRO: KIND_GYRO,
    ID_ANGLE: KIND_ANGLE,
    ID_MAG: KIND_MAG,
    ID_COMBINED: KIND_COMBINED,
}

# Field order inside each payload type.
_FIELDS = {
    ID_ACCEL: ("ax", "ay", "az"),
    ID_GYRO: ("gx", "gy", "gz"),
    ID_ANGLE: ("roll", "pitch", "yaw"),
    ID_MAG: ("mx", "my", "mz"),
    ID_COMBINED: ("ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw"),
}


@dataclass(frozen=True)
class Scales:
    """Full-scale ranges used to convert int16 counts to engineering units.

    WitMotion parts ship with different ranges.  These defaults match the
    common WT9011DCL-BT50 configuration; verify them against the datasheet of
    *your* unit (the range is also encoded in the device configuration reply).
    """

    accel_g: float = 16.0        # +/- 16 g
    gyro_dps: float = 2000.0     # +/- 2000 deg/s
    angle_deg: float = 180.0     # +/- 180 deg


@dataclass
class ParserStats:
    bytes_in: int = 0
    frames_ok: int = 0
    bad_checksum: int = 0
    bad_header: int = 0
    discarded_bytes: int = 0
    buffer_overflows: int = 0
    frames_by_id: Dict[int, int] = field(default_factory=dict)

    def observe(self, ident: int) -> None:
        self.frames_by_id[ident] = self.frames_by_id.get(ident, 0) + 1

    def describe(self) -> str:
        ids = ", ".join(f"0x{k:02X}:{v}" for k, v in sorted(self.frames_by_id.items()))
        return (
            f"bytes={self.bytes_in} ok={self.frames_ok} bad_sum={self.bad_checksum} "
            f"bad_hdr={self.bad_header} dropped={self.discarded_bytes} "
            f"overflows={self.buffer_overflows} [{ids}]"
        )


class WitParser:
    """Incremental, resynchronising parser for WitMotion serial/BLE frames."""

    def __init__(
        self,
        scales: Scales = Scales(),
        *,
        split_frames: bool = True,
        combined_frames: bool = True,
        combined_checksum: bool = False,
        max_buffer: int = 8192,
    ) -> None:
        self.scales = scales
        self.split_frames = split_frames
        self.combined_frames = combined_frames
        self.combined_checksum = combined_checksum
        self.max_buffer = max(max_buffer, COMBINED_LEN * 2)
        self.stats = ParserStats()
        self._buf = bytearray()
        self._seq = 0

    # ------------------------------------------------------------------ feed
    def feed(self, chunk: bytes, *, now: Optional[float] = None) -> List[ImuSample]:
        if now is None:
            now = time.monotonic()
        wall = time.time()

        self.stats.bytes_in += len(chunk)
        self._buf.extend(chunk)

        out: List[ImuSample] = []
        while True:
            idx = self._buf.find(HEADER)
            if idx < 0:
                # Keep the last byte: it may be the first byte of a header that
                # is split across two notifications.
                keep = 1 if self._buf else 0
                self.stats.discarded_bytes += len(self._buf) - keep
                if keep:
                    del self._buf[: len(self._buf) - 1]
                else:
                    self._buf.clear()
                break

            if idx > 0:
                self.stats.discarded_bytes += idx
                del self._buf[:idx]

            if len(self._buf) < 2:
                break

            ident = self._buf[1]
            meta = self._frame_meta(ident)
            if meta is None:
                self.stats.bad_header += 1
                self.stats.discarded_bytes += 1
                del self._buf[:1]
                continue

            length, has_checksum = meta
            if len(self._buf) < length:
                break  # wait for the rest of the frame

            frame = bytes(self._buf[:length])
            if has_checksum and (sum(frame[:-1]) & 0xFF) != frame[-1]:
                self.stats.bad_checksum += 1
                self.stats.discarded_bytes += 1
                del self._buf[:1]  # resync from the next byte
                continue

            sample = self._decode(frame, ident, now, wall)
            if sample is None:
                self.stats.bad_header += 1
                self.stats.discarded_bytes += 1
                del self._buf[:1]
                continue

            self._seq += 1
            sample.seq = self._seq
            self.stats.frames_ok += 1
            self.stats.observe(ident)
            out.append(sample)
            del self._buf[:length]

        if len(self._buf) > self.max_buffer:
            self.stats.buffer_overflows += 1
            # Keep only the tail; a stuck stream must never grow without bound.
            del self._buf[: len(self._buf) - 1]

        return out

    # --------------------------------------------------------------- helpers
    def _frame_meta(self, ident: int) -> Optional[tuple]:
        if ident == ID_COMBINED:
            if not self.combined_frames:
                return None
            # 20 bytes = no room for a checksum; a 21-byte variant is common on
            # some firmware revisions, controlled by ``combined_checksum``.
            return (COMBINED_LEN + (1 if self.combined_checksum else 0), self.combined_checksum)
        if ident in (ID_ACCEL, ID_GYRO, ID_ANGLE, ID_MAG):
            if not self.split_frames:
                return None
            return (SPLIT_LEN, True)
        return None

    def _decode(self, frame: bytes, ident: int, now: float, wall: float) -> Optional[ImuSample]:
        fields = _FIELDS.get(ident)
        if fields is None:
            return None

        count = len(fields)
        payload = frame[2: 2 + 2 * count]
        try:
            raw = struct.unpack("<%dh" % count, payload)
        except struct.error:
            return None

        sample = ImuSample(kind=_ID_TO_KIND[ident], t_mono=now, t_wall=wall, raw=frame)
        sc = self.scales

        # Bytes 8..9 carry temperature on 0x51/0x52/0x54 and a version word on
        # 0x53, so only the former are converted.
        if count == 3 and ident in (ID_ACCEL, ID_GYRO, ID_MAG):
            try:
                sample.temp_c = struct.unpack("<h", frame[8:10])[0] / 100.0
            except struct.error:
                pass

        for name, value in zip(fields, raw):
            if name in ("ax", "ay", "az"):
                setattr(sample, name, value / _FULL_SCALE * sc.accel_g)
            elif name in ("gx", "gy", "gz"):
                setattr(sample, name, value / _FULL_SCALE * sc.gyro_dps)
            elif name in ("roll", "pitch", "yaw"):
                setattr(sample, name, value / _FULL_SCALE * sc.angle_deg)
            else:  # magnetic: raw counts
                setattr(sample, name, float(value))
        return sample


# ------------------------------------------------------------------ encoding
def _to_raw(value: float, full_scale: Optional[float]) -> int:
    if full_scale is None:
        raw = int(round(value))
    else:
        raw = int(round(value / full_scale * _FULL_SCALE))
    return max(-32768, min(32767, raw))


def encode_split(ident: int, x: float, y: float, z: float, scales: Scales = Scales()) -> bytes:
    if ident == ID_ACCEL:
        fs = scales.accel_g
    elif ident == ID_GYRO:
        fs = scales.gyro_dps
    elif ident == ID_ANGLE:
        fs = scales.angle_deg
    elif ident == ID_MAG:
        fs = None
    else:
        raise ValueError(f"not a split frame id: 0x{ident:02X}")

    body = struct.pack("<hhh", _to_raw(x, fs), _to_raw(y, fs), _to_raw(z, fs))
    body += b"\x00\x00"  # temperature / version placeholder
    frame = bytes([HEADER, ident]) + body
    return frame + bytes([sum(frame) & 0xFF])


def encode_combined(
    ax: float, ay: float, az: float,
    gx: float, gy: float, gz: float,
    roll: float, pitch: float, yaw: float,
    scales: Scales = Scales(),
) -> bytes:
    payload = struct.pack(
        "<9h",
        _to_raw(ax, scales.accel_g),
        _to_raw(ay, scales.accel_g),
        _to_raw(az, scales.accel_g),
        _to_raw(gx, scales.gyro_dps),
        _to_raw(gy, scales.gyro_dps),
        _to_raw(gz, scales.gyro_dps),
        _to_raw(roll, scales.angle_deg),
        _to_raw(pitch, scales.angle_deg),
        _to_raw(yaw, scales.angle_deg),
    )
    return bytes([HEADER, ID_COMBINED]) + payload


def parse_hex_text(text: str) -> bytes:
    """Turn ``"55 61 AA .."`` (spaces, commas, 0x prefixes, newlines) into bytes."""

    cleaned = (
        text.replace(",", " ")
        .replace("0x", " ")
        .replace("0X", " ")
        .replace("\t", " ")
    )
    tokens = [t for t in cleaned.split() if t]
    out = bytearray()
    for tok in tokens:
        if len(tok) == 1:
            tok = "0" + tok
        if len(tok) != 2:
            continue
        try:
            out.append(int(tok, 16))
        except ValueError:
            continue
    return bytes(out)
