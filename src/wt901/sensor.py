"""Public SDK: ``WT901`` (blocking) and ``AsyncWT901`` (asyncio).

Both expose the same surface::

    with WT901(WT901Config(address="AA:BB:CC:DD:EE:FF")) as imu:
        reading = imu.read(timeout=2.0)
        print(reading.pitch)

    async with AsyncWT901(cfg) as imu:
        async for reading in imu.stream():
            ...

The async class owns the transport, the parser, the reconnect loop and the
calibration state.  The sync class runs that same object on a private thread
with its own event loop, because most product code is a plain blocking loop
(PLC bridge, MFC/Qt worker, service thread) and should not have to be
rewritten around asyncio.

Threading contract
------------------
* ``AsyncWT901``: single event loop, call from that loop only.
* ``WT901``: methods may be called from any thread **except** the sensor's own
  listener thread (calling ``stop()`` from inside a listener deadlocks).
  Listeners registered through ``add_listener`` run on the sensor thread.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Tuple

from .errors import (
    CalibrationError,
    ConfigError,
    ConnectionTimeout,
    NoDataError,
    NotStartedError,
    WT901Error,
)
from .model import ImuSample
from .protocol import Scales, WitParser
from .state import TelemetryBuffer

Listener = Callable[["ImuReading"], None]
ErrorListener = Callable[[BaseException], None]


class LinkState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    STOPPED = "stopped"


# --------------------------------------------------------------------- config
@dataclass
class WT901Config:
    """Everything a product needs to point the SDK at a device.

    ``transport`` exists so tests (and offline replay) can inject a simulated
    source; leave it ``None`` in production to use BLE.
    """

    address: Optional[str] = None          # "AA:BB:CC:DD:EE:FF" from `wt901 scan`
    name_contains: str = "WT901"           # fallback discovery filter
    notify_uuid: Optional[str] = None      # pin it once known (see `wt901 probe`)
    write_uuid: Optional[str] = None       # only needed for register commands
    profile: Optional["BleProfile"] = None  # known GATT layout, see wt901.ble
    emit_on: str = "attitude"              # "attitude" | "any"
    scales: Scales = field(default_factory=Scales)
    combined_checksum: bool = False
    reconnect: bool = True
    reconnect_delay_s: float = 2.0
    reconnect_max_s: float = 15.0
    connect_timeout_s: float = 25.0        # scan + first connection on Windows
    scan_timeout_s: float = 8.0
    queue_size: int = 256
    transport: Any = None                  # injected transport (tests / replay)

    def validate(self) -> None:
        if self.emit_on not in ("attitude", "any"):
            raise ConfigError(f"emit_on must be 'attitude' or 'any', got {self.emit_on!r}")
        if self.transport is None and not (self.address or self.name_contains):
            raise ConfigError("set address= or name_contains=, or inject a transport")
        if self.reconnect_delay_s <= 0:
            raise ConfigError("reconnect_delay_s must be > 0")
        if self.reconnect_max_s < self.reconnect_delay_s:
            raise ConfigError("reconnect_max_s must be >= reconnect_delay_s")
        if self.connect_timeout_s <= 0:
            raise ConfigError("connect_timeout_s must be > 0")
        if self.queue_size < 1:
            raise ConfigError("queue_size must be >= 1")

    def build_transport(self):
        if self.transport is not None:
            return self.transport
        from .transport import BleTransport

        return BleTransport(
            address=self.address,
            # Keep the name even with an address: it is the fallback when the
            # remembered address no longer resolves.
            name_contains=self.name_contains,
            notify_uuid=self.notify_uuid,
            write_uuid=self.write_uuid,
            profile=self.profile,
            connect_timeout=self.connect_timeout_s,
            scan_timeout=self.scan_timeout_s,
        )


# -------------------------------------------------------------------- reading
@dataclass(slots=True, frozen=True)
class ImuReading:
    """One consistent snapshot in engineering units.

    Angles are degrees, acceleration g, angular rate deg/s.  Values have the
    configured calibration applied (gyro bias and angle zero offset).
    """

    t_mono: float
    t_wall: float
    seq: int
    roll: float
    pitch: float
    yaw: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temp_c: Optional[float] = None
    has_attitude: bool = True  # False when the device sends no angle frames

    def as_dict(self) -> Dict[str, Any]:
        return {
            "t_mono": round(self.t_mono, 6),
            "t_wall": round(self.t_wall, 3),
            "seq": self.seq,
            "roll": round(self.roll, 3),
            "pitch": round(self.pitch, 3),
            "yaw": round(self.yaw, 3),
            "ax": round(self.ax, 4),
            "ay": round(self.ay, 4),
            "az": round(self.az, 4),
            "gx": round(self.gx, 3),
            "gy": round(self.gy, 3),
            "gz": round(self.gz, 3),
            "temp_c": None if self.temp_c is None else round(self.temp_c, 2),
            "has_attitude": self.has_attitude,
        }

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), ensure_ascii=False)

    def __str__(self) -> str:
        return (
            f"roll={self.roll:7.2f} pitch={self.pitch:7.2f} yaw={self.yaw:7.2f} | "
            f"a=({self.ax:5.2f},{self.ay:5.2f},{self.az:5.2f})g "
            f"w=({self.gx:7.2f},{self.gy:7.2f},{self.gz:7.2f})dps"
        )


@dataclass(slots=True, frozen=True)
class SensorStatus:
    state: LinkState
    address: str
    notify_uuid: Optional[str]
    frames: int
    bad_checksum: int
    bad_header: int
    rate_hz: float
    last_reading_age_s: Optional[float]
    reconnects: int
    device_name: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "device_name": self.device_name,
            "address": self.address,
            "notify_uuid": self.notify_uuid,
            "frames": self.frames,
            "bad_checksum": self.bad_checksum,
            "bad_header": self.bad_header,
            "rate_hz": round(self.rate_hz, 2),
            "last_reading_age_s": (
                None if self.last_reading_age_s is None else round(self.last_reading_age_s, 3)
            ),
            "reconnects": self.reconnects,
        }


# ---------------------------------------------------------------------- async
class AsyncWT901:
    """Asyncio sensor handle.  Use as an async context manager."""

    def __init__(self, config: Optional[WT901Config] = None) -> None:
        self.config = config or WT901Config()
        self.config.validate()

        self._transport = self.config.build_transport()
        self._parser = WitParser(
            scales=self.config.scales, combined_checksum=self.config.combined_checksum
        )
        self._buffer = TelemetryBuffer()

        self._queue: asyncio.Queue = asyncio.Queue(maxsize=self.config.queue_size)
        self._stop = asyncio.Event()
        self._connected = asyncio.Event()
        self._link_task: Optional[asyncio.Task] = None

        self._state = LinkState.DISCONNECTED
        self._latest: Optional[ImuReading] = None
        self._frames = 0
        self._reconnects = 0
        self._rate_marks: Deque[float] = deque(maxlen=400)

        self._listeners: List[Listener] = []
        self._error_listeners: List[ErrorListener] = []

        # Calibration state (applied to every reading).
        self._gyro_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._angle_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------- lifecycle
    async def start(self) -> "AsyncWT901":
        if self._link_task is not None and not self._link_task.done():
            return self
        self._stop.clear()
        self._state = LinkState.CONNECTING
        self._link_task = asyncio.create_task(self._link_loop(), name="wt901-link")
        return self

    async def wait_connected(self, timeout: Optional[float] = None) -> None:
        try:
            await asyncio.wait_for(self._connected.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            raise ConnectionTimeout(
                f"no data from the sensor within {timeout}s"
                if timeout
                else "connection timeout"
            ) from None

    async def stop(self) -> None:
        self._stop.set()
        self._state = LinkState.STOPPED
        self._connected.clear()
        if self._link_task is not None:
            self._link_task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._link_task
            self._link_task = None
        with contextlib.suppress(Exception):
            await self._transport.close()
        self._drain_queue()

    async def __aenter__(self) -> "AsyncWT901":
        await self.start()
        try:
            await self.wait_connected(self.config.connect_timeout_s)
        except BaseException:
            await self.stop()
            raise
        return self

    async def __aexit__(self, *exc_info) -> None:
        await self.stop()

    # ---------------------------------------------------------------- reading
    async def read(self, timeout: Optional[float] = None) -> ImuReading:
        """Return the next reading; raises :class:`NoDataError` on timeout.

        With ``timeout=None`` it blocks until data arrives or the sensor is
        stopped, whichever comes first (checked every 0.5 s so ``stop()`` from
        another thread cannot leave the caller hanging).
        """

        if timeout is None:
            while not self._stop.is_set():
                try:
                    return await asyncio.wait_for(self._queue.get(), timeout=0.5)
                except asyncio.TimeoutError:
                    continue
            raise NoDataError("sensor stopped")

        try:
            return await asyncio.wait_for(self._queue.get(), timeout=timeout)
        except asyncio.TimeoutError:
            raise NoDataError(f"no reading within {timeout}s") from None

    @property
    def latest(self) -> Optional[ImuReading]:
        """Most recent reading without blocking (``None`` before first data)."""

        return self._latest

    async def stream(self) -> Any:
        """Async iterator over readings; ends after :meth:`stop`."""

        while not self._stop.is_set():
            try:
                reading = await asyncio.wait_for(self._queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            yield reading

    # -------------------------------------------------------------- listeners
    def add_listener(self, callback: Listener) -> Listener:
        self._listeners.append(callback)
        return callback

    def remove_listener(self, callback: Listener) -> None:
        with contextlib.suppress(ValueError):
            self._listeners.remove(callback)

    def on_error(self, callback: ErrorListener) -> ErrorListener:
        self._error_listeners.append(callback)
        return callback

    # ------------------------------------------------------------- calibration
    @property
    def gyro_bias(self) -> Tuple[float, float, float]:
        return self._gyro_bias

    @property
    def angle_offset(self) -> Tuple[float, float, float]:
        return self._angle_offset

    async def calibrate_gyro_bias(self, duration: float = 2.0) -> Tuple[float, float, float]:
        """Average the gyro while the sensor is at rest and use it as bias.

        The sensor must be still.  Existing bias is folded into the result, so
        calling this twice does not double-correct.
        """

        samples: List[Tuple[float, float, float]] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            try:
                r = await self.read(timeout=0.5)
            except NoDataError:
                continue
            samples.append((r.gx, r.gy, r.gz))
        if len(samples) < 5:
            raise CalibrationError(f"only {len(samples)} samples, cannot calibrate")

        mean = tuple(sum(s[i] for s in samples) / len(samples) for i in range(3))
        self._gyro_bias = tuple(mean[i] + self._gyro_bias[i] for i in range(3))
        return self._gyro_bias

    async def zero_here(self, *, include_yaw: bool = False, samples: int = 20) -> Tuple[float, float, float]:
        """Treat the current attitude as the zero point."""

        collected: List[ImuReading] = []
        deadline = time.monotonic() + max(2.0, samples * 0.05)
        while len(collected) < samples and time.monotonic() < deadline:
            try:
                collected.append(await self.read(timeout=0.5))
            except NoDataError:
                continue
        if len(collected) < 3:
            raise CalibrationError("not enough samples to zero")

        roll = sum(r.roll for r in collected) / len(collected)
        pitch = sum(r.pitch for r in collected) / len(collected)
        yaw = sum(r.yaw for r in collected) / len(collected) if include_yaw else 0.0
        self._angle_offset = (
            self._angle_offset[0] + roll,
            self._angle_offset[1] + pitch,
            self._angle_offset[2] + yaw,
        )
        return self._angle_offset

    def set_gyro_bias(self, x: float, y: float, z: float) -> None:
        self._gyro_bias = (float(x), float(y), float(z))

    def set_angle_offset(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        self._angle_offset = (float(roll), float(pitch), float(yaw))

    def reset_calibration(self) -> None:
        self._gyro_bias = (0.0, 0.0, 0.0)
        self._angle_offset = (0.0, 0.0, 0.0)

    def save_calibration(self, path: str) -> None:
        payload = {
            "gyro_bias": list(self._gyro_bias),
            "angle_offset": list(self._angle_offset),
            "scales": {
                "accel_g": self.config.scales.accel_g,
                "gyro_dps": self.config.scales.gyro_dps,
                "angle_deg": self.config.scales.angle_deg,
            },
        }
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)

    def load_calibration(self, path: str) -> None:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
        self._gyro_bias = tuple(float(v) for v in payload.get("gyro_bias", [0, 0, 0]))
        self._angle_offset = tuple(float(v) for v in payload.get("angle_offset", [0, 0, 0]))

    # ----------------------------------------------------------------- status
    @property
    def state(self) -> LinkState:
        return self._state

    @property
    def status(self) -> SensorStatus:
        now = time.monotonic()
        while self._rate_marks and now - self._rate_marks[0] > 1.0:
            self._rate_marks.popleft()
        device = getattr(self._transport, "device", None)
        return SensorStatus(
            state=self._state,
            address=getattr(device, "address", "") or (self.config.address or ""),
            notify_uuid=getattr(self._transport, "notify_uuid", None),
            frames=self._frames,
            bad_checksum=self._parser.stats.bad_checksum,
            bad_header=self._parser.stats.bad_header,
            rate_hz=self._rate_hz(now),
            last_reading_age_s=None if self._latest is None else now - self._latest.t_mono,
            reconnects=self._reconnects,
            device_name=getattr(device, "name", "") or "",
        )

    def _rate_hz(self, now: float) -> float:
        """Frames/second over the last second.

        Windows' monotonic clock ticks coarsely (15.6 ms), so a burst of frames
        can share one timestamp; below 100 ms of history the rate is simply not
        measurable and 0.0 is returned instead of a meaningless number.
        """

        marks = self._rate_marks
        if len(marks) < 2:
            return 0.0
        span = now - marks[0]
        if span < 0.1:
            return 0.0
        return (len(marks) - 1) / min(span, 1.0)

    # --------------------------------------------------------------- internal
    async def _link_loop(self) -> None:
        delay = self.config.reconnect_delay_s
        first = True
        while not self._stop.is_set():
            self._state = LinkState.CONNECTING if first else LinkState.RECONNECTING
            first = False
            reason = "link closed"
            try:
                await self._transport.run(self._ingest)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - any link failure -> reconnect
                reason = f"{type(exc).__name__}: {exc}"
            if self._stop.is_set():
                break

            self._connected.clear()
            self._latest = None
            self._buffer.reset()
            self._state = LinkState.RECONNECTING
            self._notify_error(WT901Error(f"link down: {reason}"))

            if not self.config.reconnect:
                self._state = LinkState.DISCONNECTED
                break

            self._reconnects += 1
            await asyncio.sleep(delay)
            delay = min(delay * 1.6, self.config.reconnect_max_s)

    def _ingest(self, chunk: bytes) -> None:
        now = time.monotonic()
        for sample in self._parser.feed(chunk, now=now):
            self._buffer.update(sample)
            self._frames += 1
            if self._should_emit(sample):
                self._publish(now)

    def _should_emit(self, sample: ImuSample) -> bool:
        if self.config.emit_on == "any":
            return True
        return sample.has_attitude()

    def _publish(self, now: float) -> None:
        snap = self._buffer.snapshot(now)
        if snap.attitude is None and self.config.emit_on != "any":
            return
        attitude = snap.attitude if snap.attitude is not None else (0.0, 0.0, 0.0)
        accel = snap.accel or (0.0, 0.0, 0.0)
        gyro = snap.gyro or (0.0, 0.0, 0.0)
        ox, oy, oz = self._angle_offset
        bx, by, bz = self._gyro_bias

        reading = ImuReading(
            t_mono=now,
            t_wall=time.time(),
            seq=self._frames,
            roll=attitude[0] - ox,
            pitch=attitude[1] - oy,
            yaw=attitude[2] - oz,
            ax=accel[0],
            ay=accel[1],
            az=accel[2],
            gx=gyro[0] - bx,
            gy=gyro[1] - by,
            gz=gyro[2] - bz,
            temp_c=self._buffer.last_sample.temp_c if self._buffer.last_sample else None,
            has_attitude=snap.attitude is not None,
        )
        self._latest = reading
        self._rate_marks.append(now)

        if not self._connected.is_set():
            self._connected.set()
            self._state = LinkState.CONNECTED

        for callback in list(self._listeners):
            try:
                callback(reading)
            except Exception as exc:  # a bad listener must not stop acquisition
                self._notify_error(exc)

        try:
            self._queue.put_nowait(reading)
        except asyncio.QueueFull:
            with contextlib.suppress(asyncio.QueueEmpty):
                self._queue.get_nowait()
                self._queue.put_nowait(reading)

    def _notify_error(self, exc: BaseException) -> None:
        for callback in list(self._error_listeners):
            with contextlib.suppress(Exception):
                callback(exc)

    def _drain_queue(self) -> None:
        with contextlib.suppress(asyncio.QueueEmpty):
            while True:
                self._queue.get_nowait()


# ----------------------------------------------------------------------- sync
class WT901:
    """Blocking wrapper around :class:`AsyncWT901` for product code.

    Starts a private thread with its own event loop; ``read()`` blocks the
    caller.  Everything else (reconnect, calibration, parsing) is identical.
    """

    def __init__(self, config: Optional[WT901Config] = None) -> None:
        self.config = config or WT901Config()
        self.config.validate()
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._sensor: Optional[AsyncWT901] = None
        self._ready = threading.Event()
        self._lock = threading.RLock()
        self._startup_error: Optional[BaseException] = None

    # ------------------------------------------------------------- lifecycle
    def start(self, wait: bool = True, timeout: Optional[float] = None) -> "WT901":
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return self
            self._ready.clear()
            self._startup_error = None
            self._thread = threading.Thread(target=self._run_loop, name="wt901-sdk", daemon=True)
            self._thread.start()
            if not self._ready.wait(10.0):
                self._thread = None
                raise WT901Error("event loop thread failed to start")
            if self._startup_error is not None:
                self._thread = None
                raise WT901Error(f"sensor thread failed: {self._startup_error}")

        assert self._sensor is not None
        self._call(self._sensor.start(), timeout=5.0)

        if wait:
            wait_s = self.config.connect_timeout_s if timeout is None else timeout
            try:
                self._call(self._sensor.wait_connected(wait_s), timeout=wait_s + 5.0)
            except Exception:
                self.stop()
                raise
        return self

    def stop(self) -> None:
        with self._lock:
            if self._thread is None:
                return
            thread, loop, sensor = self._thread, self._loop, self._sensor
            self._thread = None

        if sensor is not None and loop is not None and loop.is_running():
            with contextlib.suppress(Exception):
                self._call(sensor.stop(), timeout=5.0)
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if thread is not None:
            thread.join(5.0)
        self._loop = None
        self._sensor = None

    def __enter__(self) -> "WT901":
        return self.start()

    def __exit__(self, *exc_info) -> None:
        self.stop()

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            self._sensor = AsyncWT901(self.config)
        except BaseException as exc:  # pragma: no cover - config already validated
            self._startup_error = exc
            self._ready.set()
            return
        self._ready.set()
        try:
            loop.run_forever()
        finally:
            with contextlib.suppress(Exception):
                loop.run_until_complete(loop.shutdown_asyncgens())
            loop.close()

    def _call(self, coro, timeout: Optional[float] = None):
        if self._loop is None or not self._loop.is_running():
            raise NotStartedError("sensor is not running; call start() first")
        if threading.current_thread() is self._thread:
            raise WT901Error("do not call blocking SDK methods from a listener callback")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout)

    # --------------------------------------------------------------- reading
    def read(self, timeout: Optional[float] = None) -> ImuReading:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return self._call(self._sensor.read(timeout), None if timeout is None else timeout + 5.0)

    def stream(self) -> Iterable[ImuReading]:
        """Blocking iterator; ends when the sensor is stopped."""

        while self._thread is not None and self._thread.is_alive():
            try:
                yield self.read(timeout=None)
            except (NotStartedError, WT901Error):
                return

    @property
    def latest(self) -> Optional[ImuReading]:
        sensor = self._sensor
        return None if sensor is None else sensor.latest

    @property
    def state(self) -> LinkState:
        sensor = self._sensor
        return LinkState.DISCONNECTED if sensor is None else sensor.state

    @property
    def status(self) -> SensorStatus:
        sensor = self._sensor
        if sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return sensor.status

    # -------------------------------------------------------------- listeners
    def add_listener(self, callback: Listener) -> Listener:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return self._sensor.add_listener(callback)

    def remove_listener(self, callback: Listener) -> None:
        if self._sensor is not None:
            self._sensor.remove_listener(callback)

    def on_error(self, callback: ErrorListener) -> ErrorListener:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return self._sensor.on_error(callback)

    # ------------------------------------------------------------- calibration
    def calibrate_gyro_bias(self, duration: float = 2.0) -> Tuple[float, float, float]:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return self._call(self._sensor.calibrate_gyro_bias(duration), timeout=duration + 10.0)

    def zero_here(self, *, include_yaw: bool = False) -> Tuple[float, float, float]:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        return self._call(self._sensor.zero_here(include_yaw=include_yaw), timeout=15.0)

    def save_calibration(self, path: str) -> None:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        self._sensor.save_calibration(path)

    def load_calibration(self, path: str) -> None:
        if self._sensor is None:
            raise NotStartedError("sensor is not running; call start() first")
        self._sensor.load_calibration(path)
