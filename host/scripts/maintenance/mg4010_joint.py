#!/usr/bin/env python3
"""MG4010 Shoulder/Elbow backend maintenance.

The 0x81 operation is named software stop and is not a power-removal command.
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

from config.joints import JOINT_CONFIGS  # noqa: E402
from motion.authorization import RuntimeMode  # noqa: E402
from scripts._motion_cli_common import create_configured_runtime, positive_float  # noqa: E402


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MG4010 joint backend maintenance")
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("raw-status", "basic-parameters", "initialize", "state"):
        sub = commands.add_parser(command)
        sub.add_argument("--joint", choices=tuple(JOINT_CONFIGS), required=True)

    move = commands.add_parser("move")
    move.add_argument("--joint", choices=tuple(JOINT_CONFIGS), required=True)
    move.add_argument("--position-deg", type=finite_float, required=True)
    move.add_argument("--velocity-deg-s", type=positive_float, required=True)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm-motion", action="store_true")

    stop = commands.add_parser("software-stop")
    stop.add_argument("--joint", choices=tuple(JOINT_CONFIGS), required=True)
    stop.add_argument("--execute", action="store_true")
    stop.add_argument("--confirm-software-stop", action="store_true")
    return parser


def _validate_confirmations(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.command == "move" and args.execute != args.confirm_motion:
        parser.error("move requires --execute and --confirm-motion")
    if args.command == "software-stop" and args.execute != args.confirm_software_stop:
        parser.error("software-stop requires --execute and --confirm-software-stop")


def _joint(runtime: object, name: str):
    return runtime.shoulder_joint if name == "shoulder" else runtime.elbow_joint


def _print_raw_report(joint: object, *, emit=print) -> None:
    driver = joint.driver
    single = driver.read_single_turn_position()
    multi = driver.read_multi_turn_position_deg()
    status = driver.read_status()
    fault = driver.read_fault()
    emit(f"joint={joint.config.name} motor_id={driver.motor_id}")
    emit(f"request_id=0x{driver.request_id:03X} response_id=0x{driver.response_id:03X}")
    emit(f"single_turn={single}")
    emit(f"multi_turn_motor_deg={multi}")
    emit(f"status={status}")
    emit(f"fault={fault}")


def run(args: argparse.Namespace, *, runtime_factory=None, emit=print) -> int:
    runtime_factory = runtime_factory or create_configured_runtime
    write = args.command in ("move", "software-stop") and args.execute
    runtime = runtime_factory(RuntimeMode.MOTION if write else RuntimeMode.READ_ONLY)
    joint = _joint(runtime, args.joint)

    if args.command == "move" and not args.execute:
        position_rad = math.radians(args.position_deg)
        velocity_rad_s = math.radians(args.velocity_deg_s)
        joint.validate_position_command(position_rad, velocity_rad_s)
        emit(
            f"preview joint={args.joint} position={args.position_deg} deg "
            f"velocity={args.velocity_deg_s} deg/s; no CAN command was sent"
        )
        return 0
    if args.command == "software-stop" and not args.execute:
        emit(f"preview joint={args.joint} software stop (0x81); no CAN command was sent")
        return 0

    with runtime:
        joint = _joint(runtime, args.joint)
        if args.command == "raw-status":
            emit(str(joint.driver.read_status()))
            emit(str(joint.driver.read_fault()))
        elif args.command == "basic-parameters":
            _print_raw_report(joint, emit=emit)
            emit(f"formal_joint_config={joint.config}")
        elif args.command == "initialize":
            emit(
                "initialize reads stable absolute position into this process; "
                "it is not enable, home, or motion"
            )
            emit(str(joint.initialize()))
        elif args.command == "state":
            joint.initialize()
            emit(str(joint.get_state()))
        elif args.command == "move":
            joint.initialize()
            joint.command_position(
                math.radians(args.position_deg),
                math.radians(args.velocity_deg_s),
            )
            emit(
                f"joint={args.joint} position command accepted; mechanical arrival "
                "is not implied"
            )
        else:
            emit(f"sending joint={args.joint} software stop (0x81)")
            joint.stop()
            emit("software stop (0x81) accepted")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_confirmations(parser, args)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("MG4010 maintenance interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"MG4010 maintenance failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
