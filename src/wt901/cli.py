"""Command line entry points.

    wt901 probe                 enumerate GATT, capture and diagnose the format
    wt901 scan                  list nearby BLE devices
    wt901 run  --name WT901     live acquisition + control
    wt901 sim                   same pipeline, simulated sensor, no hardware
    wt901 analyze --file x.hex  re-parse a captured log (bring-up debugging)
    wt901 selftest              offline end-to-end check, exit code 0/1
"""

from __future__ import annotations

import argparse
import asyncio
import time
from typing import List, Optional

from . import __version__
from .app import App, AppConfig
from .controller import AttitudeController, ControlConfig
from .protocol import WitParser, parse_hex_text
from .recorder import CsvRecorder
from .sinks import CommandLogSink, ConsoleSink, ListSink, MultiSink, Sink, TcpSink


def _require_bleak() -> None:
    try:
        import bleak  # noqa: F401
    except ImportError:
        raise SystemExit(
            "bleak is not installed.\n"
            "  python -m pip install bleak\n"
            "(the parser, simulator and tests work without it)"
        )


# --------------------------------------------------------------- arg helpers
def _add_control_args(p: argparse.ArgumentParser) -> None:
    g = p.add_argument_group("control")
    g.add_argument("--axis", default="pitch", choices=("roll", "pitch", "yaw"))
    g.add_argument("--threshold", type=float, default=15.0, help="deg, default 15")
    g.add_argument("--hysteresis", type=float, default=3.0, help="deg, default 3")
    g.add_argument("--stale-timeout", type=float, default=0.5, help="s, default 0.5")
    g.add_argument("--tilt-limit", type=float, default=60.0, help="deg, default 60")
    g.add_argument("--slew", type=float, default=2.0, help="normalised units per second")
    g.add_argument("--rate", type=float, default=50.0, help="control loop Hz, default 50")


def _add_output_args(p: argparse.ArgumentParser) -> None:
    g = p.add_argument_group("output")
    g.add_argument("--csv", default=None, help="record decoded samples to CSV")
    g.add_argument("--tcp", default=None, metavar="HOST:PORT", help="stream commands over TCP")
    g.add_argument("--tcp-format", default="json", choices=("json", "text"))
    g.add_argument("--log-cmd", default=None, metavar="PATH.jsonl", help="append commands as JSONL")
    g.add_argument("--capture", default=None, metavar="PATH.bin",
                   help="save raw notification bytes, replay with `analyze`")
    g.add_argument("--quiet", action="store_true", help="suppress per-sample data lines")


def _controller_from(args) -> AttitudeController:
    return AttitudeController(
        ControlConfig(
            axis=args.axis,
            threshold_deg=args.threshold,
            hysteresis_deg=args.hysteresis,
            stale_timeout_s=args.stale_timeout,
            tilt_limit_deg=args.tilt_limit,
            slew_per_s=args.slew,
        )
    )


def _sinks_from(args) -> Optional[Sink]:
    sinks: List[Sink] = [ConsoleSink()]
    if args.log_cmd:
        sinks.append(CommandLogSink(args.log_cmd))
    if args.tcp:
        host, _, port = args.tcp.rpartition(":")
        if not host or not port.isdigit():
            raise SystemExit(f"--tcp expects HOST:PORT, got {args.tcp!r}")
        sinks.append(TcpSink(host, int(port), fmt=args.tcp_format))
    return MultiSink(sinks) if sinks else None


def _recorder_from(args) -> Optional[CsvRecorder]:
    return CsvRecorder(args.csv) if args.csv else None


# ------------------------------------------------------------------ commands
async def cmd_scan(args) -> int:
    from .transport import scan

    _require_bleak()
    print(f"scanning {args.timeout:.0f}s ...", flush=True)
    devices = await scan(timeout=args.timeout)
    if not devices:
        print("no BLE device found (adapter on? sensor powered?)")
        return 1
    for d in devices:
        print(d)
    print(f"\n{len(devices)} device(s). Use --address or --name with `wt901 run`.")
    return 0


async def cmd_run(args) -> int:
    if args.sim:
        from .simulator import SimulatedTransport

        transport = SimulatedTransport(
            hz=args.sim_hz, period_s=args.sim_period, split=args.sim_split, verbose=True
        )
    else:
        from .ble import WITMOTION_BLE
        from .transport import BleTransport

        _require_bleak()
        transport = BleTransport(
            address=args.address,
            name_contains=args.name,
            notify_uuid=args.notify_uuid,
            profile=WITMOTION_BLE,
            connect_timeout=20.0,
            scan_timeout=args.scan_timeout,
        )

    cfg = AppConfig(
        control_hz=args.rate,
        stats_interval_s=args.stats_interval,
        print_data=not args.quiet,
        dump_hex=args.dump_hex,
        capture_path=args.capture,
    )
    app = App(
        transport,
        cfg=cfg,
        parser=WitParser(combined_checksum=args.combined_checksum),
        controller=_controller_from(args),
        sinks=_sinks_from(args),
        recorder=_recorder_from(args),
    )
    print(f"wt901 {__version__} | axis={args.axis} thr={args.threshold} | Ctrl+C to stop", flush=True)
    if args.duration and args.duration > 0:
        try:
            await asyncio.wait_for(app.run(), timeout=args.duration)
        except asyncio.TimeoutError:
            print(f"\nreached --duration {args.duration:.0f}s", flush=True)
    else:
        await app.run()
    return 0


async def cmd_probe(args) -> int:
    from .probe import probe_device

    _require_bleak()
    print(f"probing for {args.duration:.0f}s (close the vendor app first) ...", flush=True)
    report = await probe_device(
        address=args.address,
        name_contains=args.name,
        duration=args.duration,
        scan_timeout=args.scan_timeout,
    )
    print(report.as_text())
    if args.save:
        if report.capture:
            with open(args.save, "wb") as fh:
                fh.write(report.capture)
            print(f"capture saved: {args.save} ({len(report.capture)} bytes)")
        else:
            print("nothing captured, nothing saved")
    if args.json:
        import json as _json

        from .probe import report_to_dict

        print(_json.dumps(report_to_dict(report), ensure_ascii=False, indent=2))
    return 0 if (report.analysis and report.analysis.frames_ok) else 2


async def cmd_analyze(args) -> int:
    data = _load_bytes(args)
    if not data:
        print("no data")
        return 1

    parser = WitParser()
    samples = parser.feed(data)

    print(f"input bytes: {len(data)}")
    print(f"parser     : {parser.stats.describe()}")

    ids = _header_histogram(data)
    if ids:
        pretty = ", ".join(f"0x{k:02X}={v}" for k, v in sorted(ids.items()))
        print(f"0x55-prefixed ids: {pretty}")
        print("  (payload bytes can coincidentally be 0x55; only the parser's"
              " frames_ok/kind counts above are authoritative)")

    if not samples:
        print(
            "\nNo frame decoded. Possible causes: different frame layout, checksum "
            "byte present/absent, or these are not WitMotion frames at all."
        )
        return 2

    print(f"\n{samples[0].kind} frames decoded, first {min(args.limit, len(samples))}:")
    print(f"{'#':>5} {'kind':9} {'roll':>8} {'pitch':>8} {'yaw':>8} "
          f"{'ax':>7} {'ay':>7} {'az':>7} {'gx':>8} {'gy':>8} {'gz':>8}")
    for s in samples[: args.limit]:
        print(
            f"{s.seq:5d} {s.kind:9} {_f(s.roll):>8} {_f(s.pitch):>8} {_f(s.yaw):>8} "
            f"{_f(s.ax):>7} {_f(s.ay):>7} {_f(s.az):>7} "
            f"{_f(s.gx):>8} {_f(s.gy):>8} {_f(s.gz):>8}"
        )
    return 0


def _f(v: Optional[float]) -> str:
    return "n/a" if v is None else f"{v:.2f}"


def _header_histogram(data: bytes) -> dict:
    hist: dict = {}
    for i in range(len(data) - 1):
        if data[i] == 0x55:
            ident = data[i + 1]
            hist[ident] = hist.get(ident, 0) + 1
    return hist


def _load_bytes(args) -> bytes:
    if args.hex:
        return parse_hex_text(args.hex)
    if not args.file:
        raise SystemExit("give --hex '<bytes>' or --file <path>")
    with open(args.file, "rb") as fh:
        raw = fh.read()
    if args.file.lower().endswith((".hex", ".txt", ".log")):
        return parse_hex_text(raw.decode("utf-8", errors="ignore"))
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw
    if text.strip() and all(32 <= ord(ch) < 127 or ch in "\r\n\t" for ch in text[:512]):
        parsed = parse_hex_text(text)
        if len(parsed) * 2 >= len(raw) * 0.5:
            return parsed
    return raw


async def cmd_selftest(args) -> int:
    from .simulator import SimulatedTransport

    sink = ListSink()
    app = App(
        SimulatedTransport(hz=60.0, period_s=2.0),
        cfg=AppConfig(control_hz=50.0, stats_interval_s=1e9, print_data=False),
        sinks=sink,
    )
    started = time.monotonic()
    try:
        await asyncio.wait_for(app.run(), timeout=args.seconds)
    except (asyncio.TimeoutError, asyncio.CancelledError):
        pass
    elapsed = time.monotonic() - started

    names = sink.names
    checks = [
        ("pipeline ran", elapsed > 0.5),
        ("commands emitted", len(names) > 0),
        ("FWD reached", "FWD" in names),
        ("REV reached", "REV" in names),
        ("no crash", True),
    ]
    for label, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}")
    print(f"  commands: {names[:8]}{' ...' if len(names) > 8 else ''}")
    return 0 if all(ok for _, ok in checks) else 1


# --------------------------------------------------------------------- parser
def cmd_gui(args) -> int:
    """Sync entry point: Qt owns the main loop here, so no asyncio.run."""

    from .gui import run_gui

    return run_gui(
        address=args.address,
        name=args.name,
        sim=args.sim,
        screenshot=args.screenshot,
        screenshot_delay=args.screenshot_delay,
        seconds=args.duration,
        sim_amp=args.sim_amp,
        sim_period=args.sim_period,
    )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="wt901",
        description="WT9011DCL-BT50 (BLE IMU) acquisition and attitude control",
    )
    p.add_argument("--version", action="version", version=f"wt901-control {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("scan", help="list nearby BLE devices")
    sp.add_argument("--timeout", type=float, default=8.0)
    sp.set_defaults(func=cmd_scan)

    sp = sub.add_parser("probe", help="enumerate GATT, capture and diagnose the data format")
    sp.add_argument("--address", default=None, help="BLE address from `wt901 scan`")
    sp.add_argument("--name", default="WT901", help="name substring, default WT901")
    sp.add_argument("--duration", type=float, default=6.0, help="capture seconds, default 6")
    sp.add_argument("--scan-timeout", type=float, default=8.0)
    sp.add_argument("--save", default=None, metavar="PATH.bin", help="write the captured bytes")
    sp.add_argument("--json", action="store_true", help="also print the report as JSON")
    sp.set_defaults(func=cmd_probe)

    sp = sub.add_parser("run", help="connect, decode and control")
    sp.add_argument("--address", default=None, help="BLE address from `wt901 scan`")
    sp.add_argument("--name", default="WT901", help="name substring, default WT901")
    sp.add_argument("--notify-uuid", default=None)
    sp.add_argument("--scan-timeout", type=float, default=8.0)
    sp.add_argument("--combined-checksum", action="store_true",
                    help="treat the 20-byte 0x61 frame as having a trailing checksum (21 bytes)")
    sp.add_argument("--stats-interval", type=float, default=2.0)
    sp.add_argument("--dump-hex", action="store_true", help="print raw notification bytes")
    sp.add_argument("--sim", action="store_true", help="use the simulated sensor")
    sp.add_argument("--sim-hz", type=float, default=60.0)
    sp.add_argument("--sim-period", type=float, default=2.0)
    sp.add_argument("--sim-split", action="store_true", help="simulate 0x51/0x52/0x53 frames")
    sp.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    _add_control_args(sp)
    _add_output_args(sp)
    sp.set_defaults(func=cmd_run)

    sp = sub.add_parser("sim", help="alias of `run --sim`")
    sp.add_argument("--address", default=None)
    sp.add_argument("--name", default="WT901")
    sp.add_argument("--notify-uuid", default=None)
    sp.add_argument("--scan-timeout", type=float, default=8.0)
    sp.add_argument("--combined-checksum", action="store_true")
    sp.add_argument("--stats-interval", type=float, default=2.0)
    sp.add_argument("--dump-hex", action="store_true")
    sp.add_argument("--sim-hz", type=float, default=60.0)
    sp.add_argument("--sim-period", type=float, default=2.0)
    sp.add_argument("--sim-split", action="store_true")
    sp.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    _add_control_args(sp)
    _add_output_args(sp)
    sp.set_defaults(func=cmd_run, sim=True)

    sp = sub.add_parser("analyze", help="decode a captured hex/binary log")
    sp.add_argument("--hex", default=None)
    sp.add_argument("--file", default=None)
    sp.add_argument("--limit", type=int, default=20)
    sp.set_defaults(func=cmd_analyze)

    sp = sub.add_parser("selftest", help="offline end-to-end check")
    sp.add_argument("--seconds", type=float, default=2.6)
    sp.set_defaults(func=cmd_selftest)

    sp = sub.add_parser("gui", help="prototype needle-angle GUI (PySide6)")
    sp.add_argument("--address", default=None, help="BLE address from `wt901 scan`")
    sp.add_argument("--name", default="WT901", help="name substring, default WT901")
    sp.add_argument("--sim", action="store_true", help="simulated sensor, no hardware")
    sp.add_argument("--sim-amp", type=float, default=18.0, help="simulated tilt amplitude (deg)")
    sp.add_argument("--sim-period", type=float, default=6.0, help="simulated tilt period (s)")
    sp.add_argument("--screenshot", default=None, help="save a PNG and exit")
    sp.add_argument("--screenshot-delay", type=float, default=2.5)
    sp.add_argument("--duration", type=float, default=0.0, help="auto-quit after N seconds")
    sp.set_defaults(func=cmd_gui)
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.cmd == "gui":
            return cmd_gui(args)  # Qt owns the loop; no asyncio.run
        return asyncio.run(args.func(args))
    except KeyboardInterrupt:
        print("\nstopped by user", flush=True)
        return 130
