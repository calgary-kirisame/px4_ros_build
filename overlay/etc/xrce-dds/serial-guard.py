#!/usr/bin/env python3
import fcntl
import os
import struct
import sys
import termios
import time


STOP_STAMP = "/run/xrce-dds-agent.stopped"
FIONREAD = 0x541B


def _available(fd: int) -> int:
    try:
        return struct.unpack("I", fcntl.ioctl(fd, FIONREAD, struct.pack("I", 0)))[0]

    except OSError:
        return -1


def _flush(device: str, reason: str) -> None:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    try:
        before = _available(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        after = _available(fd)

    finally:
        os.close(fd)

    print(f"xrce serial guard: flushed {device}; reason={reason}; fionread={before}->{after}")


def _pre_start(device: str, threshold_s: float) -> int:
    try:
        with open(STOP_STAMP, "r", encoding="ascii") as stamp:
            stopped_at = float(stamp.read().strip())

        age_s = time.monotonic() - stopped_at

    except (FileNotFoundError, ValueError):
        _flush(device, "no-stop-stamp")
        return 0

    if age_s < threshold_s:
        wait_s = threshold_s - age_s
        print(f"xrce serial guard: waiting {wait_s:.1f}s; agent-down={age_s:.1f}s threshold={threshold_s:.1f}s")
        time.sleep(wait_s)
        age_s = time.monotonic() - stopped_at

    _flush(device, f"agent-down-{age_s:.1f}s")

    return 0


def _post_stop() -> int:
    with open(STOP_STAMP, "w", encoding="ascii") as stamp:
        stamp.write(f"{time.monotonic():.6f}\n")

    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} pre-start DEVICE [THRESHOLD_S] | post-stop", file=sys.stderr)
        return 2

    if sys.argv[1] == "pre-start" and len(sys.argv) in (3, 4):
        threshold_s = float(sys.argv[3]) if len(sys.argv) == 4 else 35.0
        return _pre_start(sys.argv[2], threshold_s)

    if sys.argv[1] == "post-stop" and len(sys.argv) == 2:
        return _post_stop()

    print(f"usage: {sys.argv[0]} pre-start DEVICE [THRESHOLD_S] | post-stop", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
