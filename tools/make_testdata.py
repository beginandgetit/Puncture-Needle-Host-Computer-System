"""Generate the test-data package: decode captures, synthesise a motion set,
and emit golden protocol vectors.

    python tools/make_testdata.py --real docs/capture_wt901ble67.bin --out dist/测试数据

Everything written here is reproducible from this script, so the delivered data
can be regenerated or extended later.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import List

from wt901.protocol import (
    ID_ACCEL,
    ID_ANGLE,
    ID_GYRO,
    Scales,
    WitParser,
    encode_combined,
    encode_split,
)

CSV_HEADER = [
    "index",
    "t_est_s",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "accel_norm_g",
]


def _f(v) -> str:
    return "" if v is None else f"{v:.4f}"


def decode_capture(data: bytes, csv_path: Path, rate_hz: float | None) -> dict:
    parser = WitParser()
    samples = parser.feed(data)

    with csv_path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_HEADER)
        for i, s in enumerate(samples):
            t = (i / rate_hz) if rate_hz else ""
            norm = None
            if None not in (s.ax, s.ay, s.az):
                norm = math.sqrt(s.ax**2 + s.ay**2 + s.az**2)
            writer.writerow(
                [i, _f(t), _f(s.roll), _f(s.pitch), _f(s.yaw),
                 _f(s.ax), _f(s.ay), _f(s.az), _f(s.gx), _f(s.gy), _f(s.gz), _f(norm)]
            )

    rolls = [s.roll for s in samples if s.roll is not None]
    pitches = [s.pitch for s in samples if s.pitch is not None]
    norms = [
        math.sqrt(s.ax**2 + s.ay**2 + s.az**2)
        for s in samples
        if None not in (s.ax, s.ay, s.az)
    ]
    duration = (len(samples) / rate_hz) if rate_hz else None

    return {
        "frames": len(samples),
        "bytes": len(data),
        "duration_s": round(duration, 2) if duration else None,
        "rate_hz": rate_hz,
        "bad_checksum": parser.stats.bad_checksum,
        "bad_header": parser.stats.bad_header,
        "roll_mean_deg": round(statistics.fmean(rolls), 3) if rolls else None,
        "roll_std_deg": round(statistics.pstdev(rolls), 3) if len(rolls) > 1 else None,
        "pitch_mean_deg": round(statistics.fmean(pitches), 3) if pitches else None,
        "pitch_std_deg": round(statistics.pstdev(pitches), 3) if len(pitches) > 1 else None,
        "accel_norm_mean_g": round(statistics.fmean(norms), 4) if norms else None,
        "accel_norm_std_g": round(statistics.pstdev(norms), 4) if len(norms) > 1 else None,
    }


def _rad(d: float) -> float:
    return math.radians(d)


def synthesise_motion(bin_path: Path, csv_path: Path, *, seconds: float = 30.0, hz: float = 100.0) -> dict:
    """Deterministic tilt sweep: roll ±6 deg (8 s), pitch ±12 deg (12 s)."""

    frames: List[bytes] = []
    n = int(seconds * hz)
    for i in range(n):
        t = i / hz
        roll = 6.0 * math.sin(2 * math.pi * t / 8.0)
        pitch = 12.0 * math.sin(2 * math.pi * t / 12.0)
        yaw = (20.0 * t) % 360.0 - 180.0

        d_roll = 6.0 * (2 * math.pi / 8.0) * math.cos(2 * math.pi * t / 8.0)
        d_pitch = 12.0 * (2 * math.pi / 12.0) * math.cos(2 * math.pi * t / 12.0)

        pr, rr = _rad(pitch), _rad(roll)
        ax = -math.sin(pr)
        ay = math.sin(rr) * math.cos(pr)
        az = math.cos(pr) * math.cos(rr)
        frames.append(encode_combined(ax, ay, az, d_roll, d_pitch, 0.0, roll, pitch, yaw))

    data = b"".join(frames)
    bin_path.write_bytes(data)
    return decode_capture(data, csv_path, hz)


def golden_vectors(path: Path) -> None:
    """Fixed frames with expected decoded values (tolerance 0.05)."""

    cases = [
        {
            "name": "combined_level",
            "frame": encode_combined(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).hex(" "),
            "expect": {"ax": 0.0, "az": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        },
        {
            "name": "combined_tilt",
            "frame": encode_combined(0.1, -0.2, 0.97, 1.0, 2.0, 3.0, 4.0, 18.0, -170.0).hex(" "),
            "expect": {"ax": 0.1, "ay": -0.2, "az": 0.97, "gx": 1.0, "gy": 2.0, "gz": 3.0,
                       "roll": 4.0, "pitch": 18.0, "yaw": -170.0},
        },
        {
            "name": "split_angle",
            "frame": encode_split(ID_ANGLE, 10.0, -5.0, 90.0).hex(" "),
            "expect": {"roll": 10.0, "pitch": -5.0, "yaw": 90.0},
        },
        {
            "name": "split_accel",
            "frame": encode_split(ID_ACCEL, 0.0, 0.0, 1.0).hex(" "),
            "expect": {"ax": 0.0, "ay": 0.0, "az": 1.0},
        },
        {
            "name": "split_gyro",
            "frame": encode_split(ID_GYRO, 100.0, -50.0, 25.0).hex(" "),
            "expect": {"gx": 100.0, "gy": -50.0, "gz": 25.0},
        },
    ]

    # verify before writing, so the file can never disagree with the parser
    for case in cases:
        parser = WitParser()
        raw = bytes.fromhex(case["frame"])
        samples = parser.feed(raw)
        assert len(samples) == 1, case["name"]
        for key, value in case["expect"].items():
            got = getattr(samples[0], key)
            assert got is not None and abs(got - value) < 0.05, (case["name"], key, got, value)

    path.write_text(
        json.dumps(
            {
                "generated_by": "tools/make_testdata.py",
                "tolerance": 0.05,
                "scales": {"accel_g": 16.0, "gyro_dps": 2000.0, "angle_deg": 180.0},
                "cases": cases,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", default="docs/capture_wt901ble67.bin")
    ap.add_argument("--out", default="dist/测试数据")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    summary = {}

    real = Path(args.real)
    if real.exists():
        stats = decode_capture(real.read_bytes(), out / "real_static_6s.csv", rate_hz=10.0)
        (out / "real_static_6s.bin").write_bytes(real.read_bytes())
        (out / "real_static_6s_stats.json").write_text(
            json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        summary["real"] = stats

    summary["synthetic"] = synthesise_motion(
        out / "synthetic_motion_30s.bin", out / "synthetic_motion_30s.csv"
    )
    golden_vectors(out / "golden_frames.json")

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
