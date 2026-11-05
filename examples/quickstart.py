"""Minimal SDK usage: connect and read five samples.

    python examples/quickstart.py --address AA:BB:CC:DD:EE:FF
    python examples/quickstart.py --sim          # no hardware needed
"""

from __future__ import annotations

import argparse
import time

from wt901 import WT901, WT901Config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default=None, help="from `python -m wt901 scan`")
    parser.add_argument("--name", default="WT901", help="name filter when no address")
    parser.add_argument("--sim", action="store_true", help="use the simulated sensor")
    args = parser.parse_args()

    if args.sim:
        from wt901.simulator import SimulatedTransport

        cfg = WT901Config(transport=SimulatedTransport(pitch_amp=20.0, period_s=4.0))
    else:
        cfg = WT901Config(address=args.address, name_contains=args.name)

    with WT901(cfg) as imu:
        print(f"connected: {imu.status.address or 'simulated'} ({imu.status.state.value})")
        for _ in range(5):
            reading = imu.read(timeout=3.0)
            print(reading)
            time.sleep(0.2)

        status = imu.status
        print(
            f"frames={status.frames} rate={status.rate_hz:.1f}Hz "
            f"bad_sum={status.bad_checksum} bad_hdr={status.bad_header}"
        )


if __name__ == "__main__":
    main()
