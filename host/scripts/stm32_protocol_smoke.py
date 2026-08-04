#!/usr/bin/env python3
"""STM32 protocol v2 smoke test; default path is strictly read-only."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.stm32_motion import (  # noqa: E402
    STM32MotionClient,
    STM32SerialConfig,
    STM32SerialTransport,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Query STM32 protocol v2. By default only VR, QS Z, QS S, QH and SQ "
            "are sent."
        )
    )
    parser.add_argument("port", help="serial port such as /dev/ttyACM0 or COM4")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="explicitly unlock optional --home/--position-mm actions",
    )
    parser.add_argument("--home", choices=("z", "slide"))
    parser.add_argument("--axis", choices=("z", "slide"), default="z")
    parser.add_argument("--position-mm", type=float)
    parser.add_argument("--speed-mm-s", type=float, default=5.0)
    parser.add_argument("--acceleration-mm-s2", type=float, default=10.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if (args.home is not None or args.position_mm is not None) and not args.allow_motion:
        parser.error("--home and --position-mm require --allow-motion")

    transport = STM32SerialTransport(
        STM32SerialConfig(
            port=args.port,
            baudrate=args.baudrate,
            timeout=min(args.timeout, 0.5),
            write_timeout=min(args.timeout, 0.5),
        )
    )
    transport.open()
    client = STM32MotionClient(transport, on_log_line=lambda line: print(f"log: {line}"))
    try:
        snapshot = client.resynchronize(timeout=args.timeout)
        print("version:", snapshot.version)
        print("Z:", snapshot.z)
        print("Slide:", snapshot.slide)
        print("homing:", snapshot.homing)
        print("suction:", snapshot.suction)

        if args.home is not None:
            print("home:", client.home(args.home, event_timeout=120.0))
        if args.position_mm is not None:
            print(
                "move:",
                client.move_absolute_mm(
                    args.axis,
                    args.position_mm,
                    args.speed_mm_s,
                    args.acceleration_mm_s2,
                    event_timeout=120.0,
                ),
            )
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
