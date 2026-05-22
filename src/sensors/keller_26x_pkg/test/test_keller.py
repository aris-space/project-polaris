"""Standalone Keller 26x sensor read loop — no ROS2 required.

Usage:
    python test_keller.py --port /dev/ttyUSB0 --rate 10
"""

import argparse
import math
import time

from keller_protocol import keller_protocol as kp

ADDRESS = 1
F73_P1 = 1       # absolute pressure channel
F73_TOB1 = 4     # water temperature channel


def connect(port: str) -> kp.KellerProtocol:
    bus = kp.KellerProtocol(port=port, baud_rate=9600, timeout=1.0, echo=False)
    bus.f48(ADDRESS)
    print(f"Connected to Keller 26x on {port}")
    return bus


def read_pressure(bus: kp.KellerProtocol) -> float:
    """Returns absolute pressure in Pa."""
    bar = bus.f73(ADDRESS, F73_P1)
    return bar * 100_000.0


def read_temperature(bus: kp.KellerProtocol) -> float:
    """Returns water temperature in °C."""
    return bus.f73(ADDRESS, F73_TOB1)


def main():
    parser = argparse.ArgumentParser(description="Keller 26x sensor read loop")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--rate", type=float, default=10.0, help="Read frequency in Hz")
    args = parser.parse_args()

    bus = connect(args.port)
    period = 1.0 / args.rate

    # Temperature is slow — read it every ~0.5 s regardless of pressure rate.
    temp_interval = 0.5
    last_temp_time = 0.0
    temperature_c = float("nan")

    print(f"Reading at {args.rate} Hz (Ctrl+C to stop)\n")
    print(f"{'Time (s)':>10}  {'P1 abs (Pa)':>14}  {'Temp (°C)':>10}")
    print("-" * 40)

    start = time.monotonic()

    try:
        while True:
            loop_start = time.monotonic()

            # Pressure
            try:
                p_pa = read_pressure(bus)
            except Exception as exc:
                print(f"  [pressure error] {exc}")
                p_pa = float("nan")

            # Temperature (throttled)
            now = time.monotonic()
            if now - last_temp_time >= temp_interval:
                try:
                    t = read_temperature(bus)
                    if isinstance(t, (int, float)) and not math.isnan(t):
                        temperature_c = t
                    else:
                        print("  [temperature] unexpected value, skipping")
                except Exception as exc:
                    print(f"  [temperature error] {exc}")
                last_temp_time = now

            elapsed = now - start
            temp_str = f"{temperature_c:>10.3f}" if not math.isnan(temperature_c) else f"{'---':>10}"
            print(f"{elapsed:>10.2f}  {p_pa:>14.2f}  {temp_str}")

            # Sleep for the remainder of the period.
            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
