"""Tiny persistent store for the last working device.

Scanning costs ~6-8 s on Windows; remembering the last address lets the next
launch connect directly (~5 s) and fall back to a scan only when that fails.
Kept deliberately small and dependency free.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


def _store_path() -> Path:
    base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
    return Path(base) / "wt901-control" / "last_device.json"


@dataclass
class LastDevice:
    address: str
    name: str = ""
    notify_uuid: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


def load_last_device() -> Optional[LastDevice]:
    path = _store_path()
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    address = str(payload.get("address", "")).strip()
    if not address:
        return None
    return LastDevice(
        address=address,
        name=str(payload.get("name", "")),
        notify_uuid=str(payload.get("notify_uuid", "")),
    )


def save_last_device(address: str, name: str = "", notify_uuid: str = "") -> None:
    if not address:
        return
    path = _store_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {"address": address, "name": name, "notify_uuid": notify_uuid}, indent=2
            ),
            encoding="utf-8",
        )
    except OSError:
        pass  # caching is best effort; never break acquisition for it


def clear_last_device() -> None:
    try:
        _store_path().unlink()
    except OSError:
        pass
