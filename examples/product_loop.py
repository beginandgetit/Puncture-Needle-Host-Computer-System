"""Product-style main loop: fixed period, explicit stale handling.

This is the shape most integrations need - a blocking loop that runs at a fixed
rate, consumes the newest reading and has a defined behaviour when data stops.
"""

from __future__ import annotations

import argparse
import time

from wt901 import NoDataError, WT901, WT901Config

PERIOD_S = 0.04  # 25 Hz application loop


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default=None)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--seconds", type=float, default=10.0, help="0 = run forever")
    args = parser.parse_args()

    if args.sim:
        from wt901.simulator import SimulatedTransport

        cfg = WT901Config(transport=SimulatedTransport(pitch_amp=20.0, period_s=6.0))
    else:
        cfg = WT901Config(address=args.address)

    deadline = time.monotonic() + args.seconds if args.seconds else None
    last_status = 0.0

    with WT901(cfg) as imu:
        try:
            while deadline is None or time.monotonic() < deadline:
                started = time.monotonic()
                try:
                    reading = imu.read(timeout=0.2)
                    handle(reading)
                except NoDataError:
                    handle_stale()

                if started - last_status >= 5.0:
                    last_status = started
                    s = imu.status
                    print(
                        f"[status] {s.state.value} frames={s.frames} "
                        f"rate={s.rate_hz:.1f}Hz reconnects={s.reconnects} "
                        f"age={s.last_reading_age_s}"
                    )

                elapsed = time.monotonic() - started
                if elapsed < PERIOD_S:
                    time.sleep(PERIOD_S - elapsed)
        except KeyboardInterrupt:
            print("stopping ...")

    safe_stop()


def handle(reading) -> None:
    # Replace with your product logic (write to PLC, update UI, log, ...).
    if abs(reading.pitch) > 15.0:
        print(f"tilt {reading.pitch:+.1f}deg -> action")


def handle_stale() -> None:
    # Data stopped: never keep the previous command alive.
    print("no data -> safe state")


def safe_stop() -> None:
    print("plant stopped")


if __name__ == "__main__":
    main()
