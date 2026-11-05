"""Command output sinks.

``send_control()`` in the original draft printed a string.  Here a sink is an
explicit interface so the acquisition code does not need to change when the
target turns out to be TCP, a serial adapter or a PLC.

All sinks are async because the TCP one needs the event loop; the console and
file sinks complete immediately.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import List, Optional, Protocol, Sequence, TextIO

from .controller import ControlCommand


class Sink(Protocol):
    async def start(self) -> None: ...
    async def emit(self, cmd: ControlCommand) -> None: ...
    async def close(self) -> None: ...


class NullSink:
    async def start(self) -> None:
        return None

    async def emit(self, cmd: ControlCommand) -> None:
        return None

    async def close(self) -> None:
        return None


class ConsoleSink:
    """Prints a line when something meaningful changes.

    "Meaningful" = command name, validity, a state-machine event (threshold
    crossing, hysteresis release, fail-safe) or a 25% move of the ramping
    value.  Plain heartbeat re-sends stay silent.
    """

    _EVENTS = (
        "threshold",
        "hysteresis_release",
        "stale",
        "tilt_limit",
        "no_attitude",
        "accel_gate",
        "link_down",
        "disconnected",
        "shutdown",
        "reset",
        "init",
    )

    def __init__(self, stream: Optional[TextIO] = None) -> None:
        self._stream = stream
        self._last = None
        self._last_value = 0.0

    async def start(self) -> None:
        self._last = None
        self._last_value = 0.0

    async def emit(self, cmd: ControlCommand) -> None:
        event = next((e for e in self._EVENTS if cmd.reason.startswith(e)), "")
        key = (cmd.name, cmd.valid, event)
        moved = abs(cmd.value - self._last_value) >= 0.25
        if key == self._last and not moved:
            return
        self._last = key
        self._last_value = cmd.value
        out = self._stream
        line = (
            f"[CTRL] {cmd.name:4s} value={cmd.value:+.2f} "
            f"reason={cmd.reason} valid={cmd.valid}"
        )
        if out is None:
            print(line, flush=True)
        else:
            out.write(line + "\n")
            out.flush()

    async def close(self) -> None:
        return None


class ListSink:
    """Keeps commands in memory; used by the self-test and unit tests."""

    def __init__(self) -> None:
        self.items: List[ControlCommand] = []

    async def start(self) -> None:
        return None

    async def emit(self, cmd: ControlCommand) -> None:
        self.items.append(cmd)

    async def close(self) -> None:
        return None

    @property
    def names(self) -> List[str]:
        return [c.name for c in self.items]


class CommandLogSink:
    """Append-only JSONL log of every emitted command (post-hysteresis)."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._fh = None

    async def start(self) -> None:
        self._fh = open(self._path, "a", encoding="utf-8", newline="\n")

    async def emit(self, cmd: ControlCommand) -> None:
        if self._fh is None:
            return
        rec = cmd.as_dict()
        rec["t_wall"] = round(time.time(), 3)
        self._fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._fh.flush()

    async def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


class TcpSink:
    """Newline-delimited command stream to a downstream controller.

    A bounded queue decouples the control loop from the socket, so a slow or
    dead peer can never stall attitude processing.  When the queue is full the
    *oldest* command is dropped, because the newest state is the one that
    matters.

    Heartbeats are sent every ``heartbeat_s`` even without a state change so
    the receiving side can implement its own watchdog.
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        heartbeat_s: float = 0.2,
        fmt: str = "json",
        queue_size: int = 64,
        connect_timeout: float = 3.0,
    ) -> None:
        self._host = host
        self._port = port
        self._heartbeat_s = heartbeat_s
        self._fmt = fmt
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=queue_size)
        self._connect_timeout = connect_timeout
        self._task: Optional[asyncio.Task] = None
        self._closed = asyncio.Event()
        self._seq = 0
        self.dropped = 0

    async def start(self) -> None:
        self._task = asyncio.create_task(self._pump(), name="tcp-sink")

    async def emit(self, cmd: ControlCommand) -> None:
        self._seq += 1
        try:
            self._queue.put_nowait((self._seq, cmd, time.time()))
        except asyncio.QueueFull:
            # Drop the oldest so the freshest command always gets through.
            try:
                self._queue.get_nowait()
                self.dropped += 1
            except asyncio.QueueEmpty:
                pass
            try:
                self._queue.put_nowait((self._seq, cmd, time.time()))
            except asyncio.QueueFull:
                self.dropped += 1

    async def close(self) -> None:
        self._closed.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None

    # --------------------------------------------------------------- internal
    def _encode(self, seq: int, cmd: ControlCommand, t_wall: float) -> bytes:
        if self._fmt == "text":
            return f"{cmd.name},{cmd.value:.3f},{seq}\n".encode("ascii")
        payload = {
            "seq": seq,
            "cmd": cmd.name,
            "value": round(cmd.value, 4),
            "valid": cmd.valid,
            "reason": cmd.reason,
            "t": round(t_wall, 3),
        }
        return (json.dumps(payload) + "\n").encode("utf-8")

    async def _pump(self) -> None:
        writer: Optional[asyncio.StreamWriter] = None
        next_heartbeat = 0.0
        last_frame = b""
        while not self._closed.is_set():
            try:
                if writer is None:
                    reader, writer = await asyncio.wait_for(
                        asyncio.open_connection(self._host, self._port),
                        timeout=self._connect_timeout,
                    )
                    next_heartbeat = time.monotonic()

                now = time.monotonic()
                if now >= next_heartbeat:
                    # Re-send the last command as a keepalive.
                    if last_frame:
                        writer.write(last_frame)
                        await writer.drain()
                    next_heartbeat = now + self._heartbeat_s

                try:
                    seq, cmd, t_wall = await asyncio.wait_for(self._queue.get(), timeout=0.05)
                except asyncio.TimeoutError:
                    continue

                frame = self._encode(seq, cmd, t_wall)
                last_frame = frame
                writer.write(frame)
                await writer.drain()
            except asyncio.CancelledError:
                raise
            except Exception:
                # Peer vanished: drop the connection and retry quietly.
                if writer is not None:
                    try:
                        writer.close()
                    except Exception:
                        pass
                    writer = None
                await asyncio.sleep(1.0)


class MultiSink:
    def __init__(self, sinks: Sequence[Sink]) -> None:
        self._sinks: List[Sink] = list(sinks)

    async def start(self) -> None:
        for s in self._sinks:
            await s.start()

    async def emit(self, cmd: ControlCommand) -> None:
        for s in self._sinks:
            try:
                await s.emit(cmd)
            except Exception:
                # One broken sink must not take down acquisition.
                continue

    async def close(self) -> None:
        for s in self._sinks:
            try:
                await s.close()
            except Exception:
                continue
