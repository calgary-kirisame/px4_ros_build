#!/usr/bin/env python3
"""Find one USB-connected PX4 and require it to be disarmed."""

from __future__ import annotations

import argparse
import sys

from pymavlink import mavutil
from serial.tools import list_ports


PX4_USB_IDS = {
    (0x26AC, 0x0010),
    (0x26AC, 0x0011),
    (0x26AC, 0x0012),
    (0x26AC, 0x0032),
    (0x3185, 0x0035),
    (0x3185, 0x0036),
    (0x3162, 0x004B),
    (0x1FC9, 0x001C),
    (0x2DAE, 0x1058),
    (0x2DAE, 0x1016),
    (0x2DAE, 0x1011),
    (0x0483, 0x5740),
    (0x1209, 0x5740),
    (0x1209, 0x5741),
    (0x3185, 0x003C),
    (0x3185, 0x0039),
    (0x3185, 0x003A),
    (0x3185, 0x003B),
    (0x2341, 0x8036),
}


def px4_ports() -> list[str]:
    return [
        port.device
        for port in list_ports.comports()
        if (port.vid, port.pid) in PX4_USB_IDS
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    ports = [args.port] if args.port else px4_ports()
    if len(ports) != 1:
        raise RuntimeError(f"expected one PX4 USB device, found {ports}")

    connection = mavutil.mavlink_connection(ports[0], baud=57600)
    heartbeat = connection.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        raise RuntimeError(f"no PX4 heartbeat on {ports[0]}")
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError("PX4 is armed")
    print(ports[0])
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
