"""Attitude -> command, reusing the SDK reading type.

The controller is deliberately separate from the sensor: it takes a Snapshot,
not a device handle, so it can be unit tested and swapped out.

    python examples/control_demo.py --sim
"""

from __future__ import annotations

import argparse
import time

from wt901 import (
    AttitudeController,
    ControlConfig,
    NoDataError,
    WT901,
    WT901Config,
)
from wt901.state import Snapshot


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default=None)
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()

    if args.sim:
        from wt901.simulator import SimulatedTransport

        cfg = WT901Config(transport=SimulatedTransport(pitch_amp=20.0, period_s=4.0))
    else:
        cfg = WT901Config(address=args.address)

    controller = AttitudeController(
        ControlConfig(axis="pitch", threshold_deg=15.0, hysteresis_deg=3.0, slew_per_s=2.0)
    )
    last = None

    with WT901(cfg) as imu:
        started = time.monotonic()
        while time.monotonic() - started < 8.0:
            try:
                reading = imu.read(timeout=0.2)
            except NoDataError:
                controller.reset()
                last = None
                print("no data -> STOP")
                continue

            command = controller.step(Snapshot.from_reading(reading), time.monotonic())
            moved = last is None or command.name != last[0] or abs(command.value - last[1]) >= 0.5
            if moved:
                last = (command.name, command.value)
                print(
                    f"{command.name:4s} value={command.value:+.2f} "
                    f"reason={command.reason:<18s} pitch={reading.pitch:+7.2f}"
                )

            # Hand `command` to your sink here, e.g.
            #   TcpSink("127.0.0.1", 9000)  ->  one JSON line per command
            #   or write a PLC register / serial frame

    print("STOP")


if __name__ == "__main__":
    main()
