#!/usr/bin/env python3
"""Feetech Rotation backend maintenance.

Rotation has no verified independent stop; this CLI intentionally has no stop command.
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

from config.project.feetech import (  # noqa: E402
    END_EFFECTOR_ROTATION_CONFIG,
    END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW,
)
from motion.authorization import RuntimeMode  # noqa: E402
from robot.feetech_rotation import build_position_payload  # noqa: E402
from scripts._motion_cli_common import create_configured_runtime  # noqa: E402


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def uint8(value: str) -> int:
    parsed = int(value, 0)
    if not 0 <= parsed <= 0xFF:
        raise argparse.ArgumentTypeError("value must fit uint8")
    return parsed


def positive_uint8(value: str) -> int:
    parsed = uint8(value)
    if parsed == 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def hex_bytes(value: str) -> bytes:
    compact = value.replace(" ", "").replace(":", "")
    try:
        data = bytes.fromhex(compact)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("data must be hexadecimal bytes") from exc
    if not data:
        raise argparse.ArgumentTypeError("data must not be empty")
    return data


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Feetech Rotation backend maintenance")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("ping")
    commands.add_parser("state")
    commands.add_parser("feedback")

    move = commands.add_parser("move")
    move.add_argument("--position-deg", type=finite_float, required=True)
    move.add_argument("--speed-raw", type=int, default=END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW)
    move.add_argument("--move-time-raw", type=uint8, default=0)
    move.add_argument("--acceleration-raw", type=uint8, default=0)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm-motion", action="store_true")
    move.add_argument("--confirm-rotation-no-stop", action="store_true")
    move.add_argument("--enable-torque", action="store_true")

    enable = commands.add_parser("torque-enable")
    enable.add_argument("--execute", action="store_true")
    enable.add_argument("--confirm-torque-enable", action="store_true")

    disable = commands.add_parser("torque-disable")
    disable.add_argument("--execute", action="store_true")
    disable.add_argument("--confirm-free-motion-risk", action="store_true")

    read = commands.add_parser("read-register")
    read.add_argument("--address", type=uint8, required=True)
    read.add_argument("--length", type=positive_uint8, required=True)

    write = commands.add_parser("write-register")
    write.add_argument("--address", type=uint8, required=True)
    write.add_argument("--data-hex", type=hex_bytes, required=True)
    write.add_argument("--execute", action="store_true")
    write.add_argument("--confirm-register-write", action="store_true")
    return parser


def _validate_confirmations(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.command == "move":
        if not args.execute and (
            args.confirm_motion or args.confirm_rotation_no_stop or args.enable_torque
        ):
            parser.error("move risk flags require --execute")
        if args.execute and not (args.confirm_motion and args.confirm_rotation_no_stop):
            parser.error("move requires motion and no-independent-stop confirmations")
    elif args.command == "torque-enable" and args.execute != args.confirm_torque_enable:
        parser.error("torque-enable requires --execute and --confirm-torque-enable")
    elif args.command == "torque-disable" and args.execute != args.confirm_free_motion_risk:
        parser.error("torque-disable requires --execute and --confirm-free-motion-risk")
    elif args.command == "write-register" and args.execute != args.confirm_register_write:
        parser.error("write-register requires --execute and --confirm-register-write")


def _preview(args: argparse.Namespace, *, emit=print) -> None:
    config = END_EFFECTOR_ROTATION_CONFIG
    if args.command == "move":
        target_raw, payload = build_position_payload(
            math.radians(args.position_deg),
            args.speed_raw,
            config,
            move_time_raw=args.move_time_raw,
            acceleration_raw=args.acceleration_raw,
        )
        emit(
            f"preview Feetech move position={args.position_deg} deg target_raw={target_raw} "
            f"payload={payload.hex()} speed_raw={args.speed_raw}; Rotation has no "
            "verified independent stop"
        )
    elif args.command == "torque-enable":
        emit("preview torque enable; this does not send a motion target")
    elif args.command == "torque-disable":
        emit("Torque disable may allow the mechanism to rotate freely or lose holding force.")
    else:
        emit(
            f"preview register write address=0x{args.address:02X} "
            f"data={args.data_hex.hex()}"
        )
    emit("preview only; no Feetech write was sent")


def run(args: argparse.Namespace, *, runtime_factory=None, emit=print) -> int:
    write = args.command in ("move", "torque-enable", "torque-disable", "write-register")
    if write and not args.execute:
        _preview(args, emit=emit)
        return 0

    runtime_factory = runtime_factory or create_configured_runtime
    runtime = runtime_factory(RuntimeMode.MOTION if write else RuntimeMode.READ_ONLY)
    with runtime:
        axis = runtime.rotation_axis
        bus = runtime.feetech_bus
        config = axis.config
        if args.command == "ping":
            bus.ping(config.servo_id)
            emit(f"servo_id={config.servo_id} ping=ok")
        elif args.command == "state":
            emit(f"config={config}")
            emit(f"position_deg={math.degrees(axis.read_position()):.6f}")
        elif args.command == "feedback":
            emit(str(axis.read_feedback()))
        elif args.command == "read-register":
            data = bus.read_registers(config.servo_id, args.address, args.length)
            emit(f"address=0x{args.address:02X} data={data.hex()}")
        elif args.command == "move":
            if args.enable_torque:
                axis.enable_torque()
                emit("explicit torque enable completed; no target was implied by enable")
            target_raw = axis.command_position(
                math.radians(args.position_deg),
                args.speed_raw,
                move_time_raw=args.move_time_raw,
                acceleration_raw=args.acceleration_raw,
            )
            emit(f"Feetech position target accepted target_raw={target_raw}")
            emit("torque state is not changed automatically after the command")
        elif args.command == "torque-enable":
            emit("Torque enable only; no motion target will be sent.")
            axis.enable_torque()
            emit("torque enable accepted")
        elif args.command == "torque-disable":
            emit("Torque disable may allow the mechanism to rotate freely or lose holding force.")
            axis.disable_torque()
            emit("torque disable accepted; this was not a stop command")
        else:
            bus.write_registers(
                config.servo_id,
                args.address,
                args.data_hex,
                expect_status=config.expect_write_status,
            )
            emit(f"register write accepted address=0x{args.address:02X}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_confirmations(parser, args)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("Feetech maintenance interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Feetech maintenance failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
