"""Acquisition worker: owns the SDK handle on a background thread.

The GUI must never block on BLE, so the worker runs the blocking SDK in its own
thread.  It deliberately does **not** emit one Qt signal per sample anymore:
a queued signal per sample turns a slow repaint into an unbounded queue and the
display falls further and further behind (the >1 s lag).

Instead the worker keeps only the newest reading in a small, lock-protected
mailbox.  The GUI polls it at its own frame rate; intermediate samples are
dropped by design.  Latency then stays bounded by one poll/paint period instead
of growing with the duration of the motion.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from PySide6.QtCore import QObject, Signal, Slot

from .. import NoDataError, WT901, WT901Config

#: Optional push signal for consumers that still want a stream.  It is capped
#: well below the sensor ODR; the GUI uses :meth:`SensorWorker.latest` instead.
PUSH_HZ = 20.0


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
        self._lock = threading.Lock()
        self._latest = None
        self._latest_seq = -1

    def stop(self) -> None:
        self._stop.set()

    def latest(self):
        """Newest reading, without blocking; ``None`` before the first sample."""

        with self._lock:
            return self._latest

    def latest_seq(self) -> int:
        with self._lock:
            return self._latest_seq

    @Slot()
    def run(self) -> None:
        """Keep trying forever; never die on a transient link problem.

        ``start(wait=False)`` returns immediately and the SDK's own link loop
        reconnects with backoff, so closing the vendor app (which holds the BLE
        link and stops the device from advertising) is enough for this window to
        recover without a restart.
        """

        imu: Optional[WT901] = None
        try:
            imu = WT901(self._config)
            imu.start(wait=False)
            last_status = 0.0
            last_push = 0.0
            while not self._stop.is_set():
                try:
                    reading = imu.read(timeout=0.2)
                except NoDataError:
                    reading = None
                if reading is not None:
                    with self._lock:
                        self._latest = reading
                        self._latest_seq = reading.seq

                    # Keep a low-rate optional push signal, but never one
                    # queued event per BLE sample.
                    now = time.monotonic()
                    if now - last_push >= 1.0 / PUSH_HZ:
                        last_push = now
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
