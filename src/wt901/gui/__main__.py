"""``python -m wt901.gui`` entry point."""

from __future__ import annotations

import argparse
from typing import List, Optional

from . import run_gui


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m wt901.gui",
        description="WT901 穿刺针角度引导样机",
    )
    parser.add_argument("--address", default=None, help="BLE 地址（见 wt901 scan）")
    parser.add_argument("--name", default="WT901", help="设备名过滤，默认 WT901")
    parser.add_argument("--sim", action="store_true", help="模拟模式，不需要硬件")
    parser.add_argument(
        "--seconds", "--duration", dest="seconds", type=float, default=0.0,
        help="N 秒后自动退出（0=一直运行）",
    )
    parser.add_argument("--screenshot", default=None, help="截图保存到指定 PNG 后退出")
    parser.add_argument("--screenshot-delay", type=float, default=2.5)
    parser.add_argument("--sim-amp", type=float, default=18.0, help="模拟倾角幅度（度）")
    parser.add_argument("--sim-period", type=float, default=6.0, help="模拟摆动周期（秒）")
    args = parser.parse_args(argv)

    return run_gui(
        address=args.address,
        name=args.name,
        sim=args.sim,
        screenshot=args.screenshot,
        screenshot_delay=args.screenshot_delay,
        seconds=args.seconds,
        sim_amp=args.sim_amp,
        sim_period=args.sim_period,
    )


if __name__ == "__main__":
    raise SystemExit(main())
