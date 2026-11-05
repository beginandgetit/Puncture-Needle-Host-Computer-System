"""Wiring: transport -> parser -> buffer -> control loop -> sinks."""

from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional

from .controller import STOP, AttitudeController, ControlCommand
from .protocol import WitParser
from .recorder import CsvRecorder
from .sinks import Sink
from .state import TelemetryBuffer


@dataclass
class AppConfig:
    control_hz: float = 50.0
    stats_interval_s: float = 2.0
    heartbeat_s: float = 0.2
    reconnect_delay_s: float = 3.0
    max_backoff_s: float = 15.0
    print_data: bool = True
    dump_hex: bool = False
    capture_path: Optional[str] = None  # raw notification bytes, for `wt901 analyze`


class App:
    def __init__(
        self,
        transport,
        *,
        cfg: AppConfig = AppConfig(),
        parser: Optional[WitParser] = None,
        buffer: Optional[TelemetryBuffer] = None,
        controller: Optional[AttitudeController] = None,
        sinks: Optional[Sink] = None,
        recorder: Optional[CsvRecorder] = None,
    ) -> None:
        self._transport = transport
        self.cfg = cfg
        self._parser = parser or WitParser()
        self._buffer = buffer or TelemetryBuffer()
        self._controller = controller or AttitudeController()
        self._sinks = sinks
        self._recorder = recorder

        self._stopping = False
        self._ctrl_task: Optional[asyncio.Task] = None
        self._capture = None
        self._last_cmd: Optional[ControlCommand] = None
        self._last_emitted: Optional[ControlCommand] = None
        self._last_emit_t = 0.0

        self._frames = 0
        self._last_arrival: Optional[float] = None
        self._intervals: Deque[float] = deque(maxlen=400)
        self._stats_frames = 0
        self._stats_t = time.monotonic()
        self._got_data = False
        self.reconnects = 0

    # ------------------------------------------------------------------- api
    async def run(self) -> None:
        if self._sinks is not None:
            await self._sinks.start()
        if self._recorder is not None:
            self._recorder.open()
        if self.cfg.capture_path:
            self._capture = open(self.cfg.capture_path, "wb")

        self._ctrl_task = asyncio.create_task(self._control_loop(), name="control-loop")
        try:
            await self._transport_loop()
        except asyncio.CancelledError:
            raise
        finally:
            self._stopping = True
            if self._ctrl_task is not None:
                self._ctrl_task.cancel()
                try:
                    await self._ctrl_task
                except (asyncio.CancelledError, Exception):
                    pass
                self._ctrl_task = None
            await self._emit(
                ControlCommand(STOP, 0.0, "shutdown", time.monotonic(), valid=True), force=True
            )
            if self._recorder is not None:
                self._recorder.close()
            if self._capture is not None:
                self._capture.close()
                self._capture = None
            if self._sinks is not None:
                await self._sinks.close()

    # ------------------------------------------------------- transport layer
    async def _transport_loop(self) -> None:
        delay = self.cfg.reconnect_delay_s
        while not self._stopping:
            self._got_data = False
            try:
                await self._transport.run(self._on_bytes)
                reason = "stream ended"
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - any link problem means reconnect
                reason = f"{type(exc).__name__}: {exc}"

            if self._stopping:
                return

            print(f"[link] down: {reason}", flush=True)
            self._buffer.reset()
            self._controller.reset()
            await self._emit(
                ControlCommand(STOP, 0.0, "link_down", time.monotonic(), valid=False), force=True
            )

            if self._got_data:
                delay = self.cfg.reconnect_delay_s
                self.reconnects += 1
            print(f"[link] reconnecting in {delay:.1f}s ...", flush=True)
            await asyncio.sleep(delay)
            delay = min(delay * 1.5, self.cfg.max_backoff_s)

    def _on_bytes(self, chunk: bytes) -> None:
        now = time.monotonic()
        if self.cfg.dump_hex:
            print(f"[raw] {chunk.hex(' ')}", flush=True)
        if self._capture is not None:
            self._capture.write(chunk)
        if self._last_arrival is not None:
            self._intervals.append(now - self._last_arrival)
        self._last_arrival = now

        for sample in self._parser.feed(chunk, now=now):
            self._got_data = True
            self._buffer.update(sample)
            self._frames += 1
            self._stats_frames += 1
            if self._recorder is not None:
                self._recorder.write(sample, self._last_cmd)

    # ---------------------------------------------------------- control loop
    async def _control_loop(self) -> None:
        period = 1.0 / max(self.cfg.control_hz, 1.0)
        next_t = time.monotonic()
        while not self._stopping:
            now = time.monotonic()
            snap = self._buffer.snapshot(now)
            cmd = self._controller.step(snap, now)
            self._last_cmd = cmd

            if self._should_emit(cmd, now):
                await self._emit(cmd)

            self._maybe_stats(now, snap)

            next_t += period
            delay = next_t - time.monotonic()
            if delay < 0:
                # Fell behind (GC pause, debugger, ...): re-base instead of
                # trying to "catch up" with a burst of iterations.
                next_t = time.monotonic()
                delay = 0.0
            await asyncio.sleep(delay)

    def _should_emit(self, cmd: ControlCommand, now: float) -> bool:
        last = self._last_emitted
        if last is None:
            return True
        if (cmd.name, cmd.reason, cmd.valid) != (last.name, last.reason, last.valid):
            return True
        if now - self._last_emit_t >= self.cfg.heartbeat_s:
            return True
        return False

    async def _emit(self, cmd: ControlCommand, *, force: bool = False) -> None:
        self._last_emitted = cmd
        self._last_emit_t = time.monotonic()
        if self._sinks is not None:
            await self._sinks.emit(cmd)

    # ---------------------------------------------------------------- status
    def _maybe_stats(self, now: float, snap) -> None:
        if now - self._stats_t < self.cfg.stats_interval_s:
            return
        elapsed = now - self._stats_t
        rate = self._stats_frames / elapsed if elapsed > 0 else 0.0
        self._stats_t = now
        self._stats_frames = 0

        jitter = ""
        if self._intervals:
            values = sorted(self._intervals)
            p95 = values[int(0.95 * (len(values) - 1))] * 1000.0
            jitter = f" jitter_p95={p95:.1f}ms"

        attitude = snap.attitude
        body = (
            f"r={attitude[0]:7.2f} p={attitude[1]:7.2f} y={attitude[2]:7.2f}"
            if attitude
            else "r=   n/a p=   n/a y=   n/a"
        )
        age = snap.age_attitude(now) * 1000.0 if attitude else float("nan")
        cmd = self._last_cmd.name if self._last_cmd else "----"
        print(
            f"[stat] {body} age={age:6.1f}ms rate={rate:6.1f}Hz{jitter} "
            f"frames={self._frames} cmd={cmd} | {self._parser.stats.describe()}",
            flush=True,
        )
        if self.cfg.print_data:
            print(
                f"[data] ax={_fmt(snap.accel, 0)} ay={_fmt(snap.accel, 1)} az={_fmt(snap.accel, 2)} "
                f"gx={_fmt(snap.gyro, 0)} gy={_fmt(snap.gyro, 1)} gz={_fmt(snap.gyro, 2)}",
                flush=True,
            )


def _fmt(vec, index: int) -> str:
    if vec is None:
        return "  n/a"
    return f"{vec[index]:6.2f}"
