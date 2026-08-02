#!/usr/bin/env python3
"""Feetech 旋转轴显式 dry-run/人工测试工具。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.feetech_protocol import (  # noqa: E402
    INSTRUCTION_WRITE,
    FeetechBus,
    FeetechSerialConfig,
    build_instruction_packet,
)
from robot.feetech_rotation import (  # noqa: E402
    FeetechRotationAxis,
    FeetechRotationConfig,
    build_position_payload,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Feetech rotation dry-run by default; --execute opens hardware"
    )
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--position-rad", type=float)
    operation.add_argument("--read-position", action="store_true")
    operation.add_argument("--disable", action="store_true")
    parser.add_argument("--servo-id", type=int, required=True)
    parser.add_argument("--counts-per-turn", type=int, required=True)
    parser.add_argument("--zero-raw", type=int, required=True)
    parser.add_argument("--direction-sign", type=int, choices=(-1, 1), required=True)
    parser.add_argument("--min-position-rad", type=float, required=True)
    parser.add_argument("--max-position-rad", type=float, required=True)
    parser.add_argument("--max-speed-raw", type=int, required=True)
    parser.add_argument("--speed-raw", type=int)
    parser.add_argument("--move-time-raw", type=int, default=0)
    parser.add_argument("--acceleration-raw", type=int, default=0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enable-torque", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument(
        "--no-expect-write-status",
        dest="expect_write_status",
        action="store_false",
        help="仅在具体型号的 Status Return Level 已确认不回复写命令时使用",
    )
    parser.set_defaults(expect_write_status=True)
    return parser


def _config(args: argparse.Namespace) -> FeetechRotationConfig:
    return FeetechRotationConfig(
        name="rotation",
        servo_id=args.servo_id,
        counts_per_turn=args.counts_per_turn,
        zero_raw=args.zero_raw,
        direction_sign=args.direction_sign,
        min_position_rad=args.min_position_rad,
        max_position_rad=args.max_position_rad,
        max_speed_raw=args.max_speed_raw,
        expect_write_status=args.expect_write_status,
    )


def _validate_cli(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if args.position_rad is not None and args.speed_raw is None:
        parser.error("--position-rad requires --speed-raw")
    if args.enable_torque and args.position_rad is None:
        parser.error("--enable-torque is only valid with --position-rad")
    if args.execute and (not args.port or args.baudrate is None):
        parser.error("--execute requires explicit --port and --baudrate")
    if not args.execute and args.enable_torque:
        parser.error("--enable-torque requires --execute")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_cli(args, parser)
    config = _config(args)

    if not args.execute:
        preview: dict[str, object] = {
            "mode": "dry-run",
            "servo_id": config.servo_id,
            "operation": "read" if args.read_position else "disable" if args.disable else "position",
        }
        if args.position_rad is not None:
            target_raw, payload = build_position_payload(
                args.position_rad,
                args.speed_raw,
                config,
                move_time_raw=args.move_time_raw,
                acceleration_raw=args.acceleration_raw,
            )
            packet = build_instruction_packet(
                config.servo_id,
                INSTRUCTION_WRITE,
                bytes((config.registers.goal_position,)) + payload,
            )
            preview.update(target_raw=target_raw, write_payload_hex=payload.hex(), packet_hex=packet.hex())
        print(json.dumps(preview, ensure_ascii=False, indent=2))
        return 0

    serial_config = FeetechSerialConfig(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        write_timeout=args.timeout,
    )
    with FeetechBus(serial_config) as bus:
        axis = FeetechRotationAxis(bus, config)
        if args.read_position:
            print(json.dumps({"position_rad": axis.read_position()}, indent=2))
        elif args.disable:
            axis.disable_torque()
            print("torque disabled")
        else:
            if args.enable_torque:
                axis.enable_torque()
            target_raw = axis.command_position(
                args.position_rad,
                args.speed_raw,
                move_time_raw=args.move_time_raw,
                acceleration_raw=args.acceleration_raw,
            )
            print(json.dumps({"commanded_target_raw": target_raw}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
