"""BLE profile, command frame and characteristic-selection tests (no hardware)."""

from __future__ import annotations

import os
import tempfile
import unittest

from wt901.ble import WITMOTION_BLE, BleProfile, DiscoveredCharacteristic, wit_frame, wit_save, wit_unlock
from wt901.devices import clear_last_device, load_last_device, save_last_device
from wt901.transport import TransportError, pick_notify_uuid, pick_write_uuid

NOTIFY = DiscoveredCharacteristic("0000ffe4-0000-1000-8000-00805f9a34fb", ("notify",))
WRITE = DiscoveredCharacteristic(
    "0000ffe9-0000-1000-8000-00805f9a34fb", ("write", "write-without-response")
)
BATTERY = DiscoveredCharacteristic("00002a19-0000-1000-8000-00805f9b34fb", ("notify",))
HM10 = DiscoveredCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb", ("notify",))


class TestProfile(unittest.TestCase):
    def test_witmotion_uuids(self) -> None:
        self.assertIn("ffe5", WITMOTION_BLE.service_uuid.lower())
        self.assertIn("ffe4", WITMOTION_BLE.notify_uuid.lower())
        self.assertIn("ffe9", WITMOTION_BLE.write_uuid.lower())
        # WitMotion uses the 9a34fb base, probes must not assume 9b34fb
        self.assertTrue(WITMOTION_BLE.notify_uuid.endswith("9a34fb"))

    def test_wit_frame_checksum(self) -> None:
        # FF AA 03 64 00 -> sum 0x10
        self.assertEqual(wit_frame(0x03, 0x64), bytes([0xFF, 0xAA, 0x03, 0x64, 0x00, 0x10]))

    def test_wit_frame_range(self) -> None:
        with self.assertRaises(ValueError):
            wit_frame(0x100)
        with self.assertRaises(ValueError):
            wit_frame(0x03, 0x10000)

    def test_unlock_and_save(self) -> None:
        self.assertEqual(wit_unlock(), bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
        self.assertEqual(len(wit_save()), 6)


class TestNotifySelection(unittest.TestCase):
    def test_explicit_wins(self) -> None:
        uuid = pick_notify_uuid([NOTIFY, BATTERY], explicit="custom-uuid")
        self.assertEqual(uuid, "custom-uuid")

    def test_profile_match(self) -> None:
        uuid = pick_notify_uuid([BATTERY, NOTIFY], profile=WITMOTION_BLE)
        self.assertEqual(uuid, NOTIFY.uuid)

    def test_profile_missing_falls_back(self) -> None:
        # Profile UUID absent -> the short-UUID hint list still finds ffe1.
        uuid = pick_notify_uuid([BATTERY, HM10], profile=WITMOTION_BLE)
        self.assertEqual(uuid, HM10.uuid)

    def test_single_candidate(self) -> None:
        self.assertEqual(pick_notify_uuid([NOTIFY]), NOTIFY.uuid)

    def test_hint_when_multiple(self) -> None:
        uuid = pick_notify_uuid([BATTERY, HM10], profile=None)
        self.assertEqual(uuid, HM10.uuid)

    def test_ambiguous_raises(self) -> None:
        other = DiscoveredCharacteristic("00001234-0000-1000-8000-00805f9b34fb", ("notify",))
        with self.assertRaises(TransportError):
            pick_notify_uuid([BATTERY, other])

    def test_no_notify_raises(self) -> None:
        with self.assertRaises(TransportError):
            pick_notify_uuid([WRITE])


class TestWriteSelection(unittest.TestCase):
    def test_profile_write(self) -> None:
        self.assertEqual(pick_write_uuid([NOTIFY, WRITE], profile=WITMOTION_BLE), WRITE.uuid)

    def test_first_writable(self) -> None:
        self.assertEqual(pick_write_uuid([NOTIFY, WRITE]), WRITE.uuid)

    def test_none_when_read_only(self) -> None:
        self.assertIsNone(pick_write_uuid([NOTIFY]))


class TestDeviceStore(unittest.TestCase):
    def setUp(self) -> None:
        self._old = os.environ.get("LOCALAPPDATA")
        self._tmp = tempfile.mkdtemp(prefix="wt901_store_")
        os.environ["LOCALAPPDATA"] = self._tmp

    def tearDown(self) -> None:
        clear_last_device()
        if self._old is None:
            os.environ.pop("LOCALAPPDATA", None)
        else:
            os.environ["LOCALAPPDATA"] = self._old

    def test_roundtrip(self) -> None:
        save_last_device("EA:D7:22:40:D3:0A", "WT901BLE67", "ffe4")
        cached = load_last_device()
        self.assertIsNotNone(cached)
        self.assertEqual(cached.address, "EA:D7:22:40:D3:0A")
        self.assertEqual(cached.name, "WT901BLE67")

    def test_missing_returns_none(self) -> None:
        self.assertIsNone(load_last_device())

    def test_clear(self) -> None:
        save_last_device("AA:BB")
        clear_last_device()
        self.assertIsNone(load_last_device())

    def test_empty_address_ignored(self) -> None:
        save_last_device("")
        self.assertIsNone(load_last_device())


if __name__ == "__main__":
    unittest.main()
