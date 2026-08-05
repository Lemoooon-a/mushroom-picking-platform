#!/usr/bin/env python3
"""STM32 machine-protocol maintenance for Slide, Z, and Vacuum.

This is a backend-maintenance entry point, not the unified manual-motion CLI.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import math
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[2]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from motion.authorization import RuntimeMode  # noqa: E402
from scripts._motion_cli_common import create_configured_runtime, positive_float  # noqa: E402


_AXES = ("slide", "z")
_AXIS_CODES = {"slide": "S", "z": "Z"}
_WRITE_COMMANDS = {
    "move",
    "home",
    "stop",
    "enable",
    "disable",
    "clear-fault",
    "suction-start",
    "suction-release",
    "suction-stop",
}


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def mm_to_um(value: float) -> int:
    """Convert finite engineering millimetres to the integer protocol unit."""

    if not math.isfinite(value):
        raise ValueError("millimetre value must be finite")
    return round(value * 1000.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 motion/vacuum backend maintenance")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("version")
    state = commands.add_parser("state")
    state.add_argument("--axis", choices=_AXES, required=True)

    move = commands.add_parser("move")
    move.add_argument("--axis", choices=_AXES, required=True)
    move.add_argument("--position-mm", type=finite_float, required=True)
    move.add_argument("--velocity-mm-s", type=positive_float, required=True)
    move.add_argument("--acceleration-mm-s2", type=positive_float, required=True)
    move.add_argument("--event-timeout", type=positive_float, default=120.0)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm-motion", action="store_true")

    home = commands.add_parser("home")
    home.add_argument("--axis", choices=_AXES, required=True)
    home.add_argument("--event-timeout", type=positive_float, default=120.0)
    home.add_argument("--execute", action="store_true")
    home.add_argument("--confirm-home-motion", action="store_true")

    confirmations = {
        "stop": "confirm-stop",
        "enable": "confirm-enable",
        "disable": "confirm-disable",
        "clear-fault": "confirm-clear-fault",
    }
    for command, confirmation in confirmations.items():
        sub = commands.add_parser(command)
        sub.add_argument("--axis", choices=_AXES, required=True)
        sub.add_argument("--execute", action="store_true")
        sub.add_argument(f"--{confirmation}", action="store_true")

    commands.add_parser("suction-state")
    for command in ("suction-start", "suction-release", "suction-stop"):
        sub = commands.add_parser(command)
        sub.add_argument("--execute", action="store_true")
        sub.add_argument("--confirm-suction-action", action="store_true")
    return parser


def _validate_confirmations(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.command == "move":
        confirmed = args.confirm_motion
    elif args.command == "home":
        confirmed = args.confirm_home_motion
    elif args.command in ("stop", "enable", "disable", "clear-fault"):
        confirmed = getattr(args, "confirm_" + args.command.replace("-", "_"))
    elif args.command.startswith("suction-") and args.command != "suction-state":
        confirmed = args.confirm_suction_action
    else:
        return
    if args.execute != confirmed:
        parser.error(f"{args.command} requires --execute and its specific confirmation")


def _preview(args: argparse.Namespace, emit=print) -> None:
    axis = getattr(args, "axis", None)
    axis_code = _AXIS_CODES.get(axis)
    if args.command == "move":
        values = (
            mm_to_um(args.position_mm),
            mm_to_um(args.velocity_mm_s),
            mm_to_um(args.acceleration_mm_s2),
        )
        emit(
            f"preview STM32 MA axis={axis} axis_code={axis_code} "
            f"engineering=(position={args.position_mm} mm, "
            f"velocity={args.velocity_mm_s} mm/s, acceleration={args.acceleration_mm_s2} mm/s²) "
            f"protocol=(position={values[0]} um, velocity={values[1]} um/s, "
            f"acceleration={values[2]} um/s²)"
        )
    else:
        machine_command = {
            "home": "HM",
            "stop": "ST",
            "enable": "EN",
            "disable": "DI",
            "clear-fault": "CF",
            "suction-start": "SU",
            "suction-release": "SR",
            "suction-stop": "SX",
        }[args.command]
        target = f"axis={axis} axis_code={axis_code}" if axis else "target=vacuum"
        emit(f"preview STM32 machine command={machine_command} {target}")
    emit("preview only; no STM32 command was sent")


def run(args: argparse.Namespace, *, runtime_factory=None, emit=print) -> int:
    if args.command in _WRITE_COMMANDS and not args.execute:
        _preview(args, emit=emit)
        return 0
    runtime_factory = runtime_factory or create_configured_runtime
    mode = RuntimeMode.MOTION if args.command in _WRITE_COMMANDS else RuntimeMode.READ_ONLY
    runtime = runtime_factory(mode)
    with runtime:
        client = runtime.stm32_client
        if args.command == "version":
            emit(str(client.version()))
        elif args.command == "state":
            emit(str(client.query_axis(args.axis)))
        elif args.command == "suction-state":
            emit(str(client.query_suction()))
        elif args.command == "move":
            position_um = mm_to_um(args.position_mm)
            velocity_um_s = mm_to_um(args.velocity_mm_s)
            acceleration_um_s2 = mm_to_um(args.acceleration_mm_s2)
            emit(
                f"sending MA axis={args.axis} axis_code={_AXIS_CODES[args.axis]} "
                f"engineering={args.position_mm} mm/"
                f"{args.velocity_mm_s} mm/s/{args.acceleration_mm_s2} mm/s² "
                f"protocol={position_um} um/{velocity_um_s} um/s/{acceleration_um_s2} um/s²"
            )
            emit(str(client.move_absolute(
                args.axis,
                position_um,
                velocity_um_s,
                acceleration_um_s2,
                event_timeout=args.event_timeout,
            )))
        elif args.command == "home":
            emit(f"sending HM axis={args.axis} axis_code={_AXIS_CODES[args.axis]}")
            emit(str(client.home(args.axis, event_timeout=args.event_timeout)))
        elif args.command in ("stop", "enable", "disable", "clear-fault"):
            method = {
                "stop": client.stop,
                "enable": client.enable,
                "disable": client.disable,
                "clear-fault": client.clear_fault,
            }[args.command]
            emit(
                f"sending {args.command} to STM32 axis={args.axis} "
                f"axis_code={_AXIS_CODES[args.axis]}"
            )
            method(args.axis)
        elif args.command == "suction-start":
            emit(str(client.suction_start()))
        elif args.command == "suction-release":
            emit(str(client.suction_release()))
        else:
            client.suction_stop()
            emit("STM32 suction stop accepted")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_confirmations(parser, args)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("STM32 maintenance interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"STM32 maintenance failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
