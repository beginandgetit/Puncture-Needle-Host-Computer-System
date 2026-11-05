"""Acquisition worker: owns the SDK handle on a background thread.

The GUI must never block on BLE, so the worker runs the blocking SDK in its own
thread and republishes readings through Qt signals (queued automatically).
"""

from __future__ import annotations

import threading
import time

from PySide6.QtCore import QObject, Signal, Slot

from .. import NoDataError, WT901, WT901Config


class SensorWorker(QObject):
    reading = Signal(object)
    status = Signal(object)
    connected = Signal(object)
    failed = Signal(str)
    finished = Signal()

    def __init__(self, config: WT901Config) -> None:
        super().__init__()
        self._config = config
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    @Slot()
    def run(self) -> None:
        """Keep trying forever; never die on a transient link problem.

        ``start(wait=False)`` returns immediately and the SDK's own link loop
        reconnects with backoff, so closing the vendor app (which holds the BLE
        link and stops the device from advertising) is enough for this window to
        recover without a restart.
        """

        imu: WT901 | None = None
        try:
            imu = WT901(self._config)
            imu.start(wait=False)
            last_status = 0.0
            while not self._stop.is_set():
                try:
                    reading = imu.read(timeout=0.2)
                except NoDataError:
                    reading = None
                if reading is not None:
                    self.reading.emit(reading)

                now = time.monotonic()
                if now - last_status >= 0.5:
                    last_status = now
                    self.status.emit(imu.status)
        except Exception as exc:  # noqa: BLE001 - config errors are fatal
            self.failed.emit(f"{type(exc).__name__}: {exc}")
        finally:
            if imu is not None:
                try:
                    imu.stop()
                except Exception:
                    pass
            self.finished.emit()
