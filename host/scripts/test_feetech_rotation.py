#!/usr/bin/env python3
"""Feetech 旋转轴显式 dry-run/人工测试工具。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.feetech_protocol import (  # noqa: E402
    INSTRUCTION_PING,
    INSTRUCTION_READ,
    INSTRUCTION_WRITE,
    FeetechBus,
    FeetechSerialConfig,
    build_instruction_packet,
)
from config.feetech import (  # noqa: E402
    END_EFFECTOR_ROTATION_CONFIG,
    END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW,
    END_EFFECTOR_ROTATION_POSITIVE_DIRECTION,
    FEETECH_MODEL_PROFILES,
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
    operation.add_argument("--ping", action="store_true")
    operation.add_argument("--read-raw-position", action="store_true")
    operation.add_argument("--position-rad", type=float)
    operation.add_argument("--position-deg", type=float)
    operation.add_argument("--read-position", action="store_true")
    operation.add_argument("--disable", action="store_true")
    parser.add_argument(
        "--model",
        choices=tuple(FEETECH_MODEL_PROFILES),
        default="sm45bl-c001",
    )
    parser.add_argument(
        "--servo-id",
        type=int,
        default=END_EFFECTOR_ROTATION_CONFIG.servo_id,
        help=f"舵机 ID（项目默认 {END_EFFECTOR_ROTATION_CONFIG.servo_id}）",
    )
    parser.add_argument(
        "--counts-per-turn",
        type=int,
        help="兼容旧命令；必须与所选型号 profile 一致",
    )
    parser.add_argument(
        "--zero-raw",
        type=int,
        default=END_EFFECTOR_ROTATION_CONFIG.zero_raw,
        help=f"逻辑零点（项目默认 {END_EFFECTOR_ROTATION_CONFIG.zero_raw}）",
    )
    parser.add_argument(
        "--direction-sign",
        type=int,
        choices=(-1, 1),
        default=END_EFFECTOR_ROTATION_CONFIG.direction_sign,
        help=(
            "raw 增量到逻辑角度的方向符号"
            f"（项目默认 {END_EFFECTOR_ROTATION_CONFIG.direction_sign}，"
            f"对应 {END_EFFECTOR_ROTATION_POSITIVE_DIRECTION}）"
        ),
    )
    parser.add_argument("--min-position-rad", type=float)
    parser.add_argument("--max-position-rad", type=float)
    parser.add_argument("--min-position-deg", type=float)
    parser.add_argument("--max-position-deg", type=float)
    parser.add_argument(
        "--limit-deg",
        type=float,
        help="以逻辑零点为中心设置对称软件限位，例如 45 表示 -45..+45 deg",
    )
    parser.add_argument(
        "--max-speed-raw",
        type=int,
        default=END_EFFECTOR_ROTATION_CONFIG.max_speed_raw,
        help=f"软件速度上限（项目默认 {END_EFFECTOR_ROTATION_CONFIG.max_speed_raw}）",
    )
    parser.add_argument(
        "--speed-raw",
        type=int,
        default=END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW,
        help=f"位置命令速度（项目默认 {END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW}）",
    )
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
    profile = FEETECH_MODEL_PROFILES[args.model]
    return profile.make_rotation_config(
        name="rotation",
        servo_id=args.servo_id,
        zero_raw=args.zero_raw,
        direction_sign=args.direction_sign,
        min_position_rad=args._resolved_min_position_rad,
        max_position_rad=args._resolved_max_position_rad,
        max_speed_raw=args._resolved_max_speed_raw,
        expect_write_status=args.expect_write_status,
    )


def _resolve_limits(
    args: argparse.Namespace, parser: argparse.ArgumentParser
) -> tuple[float, float]:
    rad_values = (args.min_position_rad, args.max_position_rad)
    deg_values = (args.min_position_deg, args.max_position_deg)
    if args.limit_deg is not None:
        if any(value is not None for value in rad_values + deg_values):
            parser.error("--limit-deg cannot be combined with min/max limit options")
        if not math.isfinite(args.limit_deg) or not 0 < args.limit_deg < 180:
            parser.error("--limit-deg must be finite and in 0..180 degrees")
        limit_rad = math.radians(args.limit_deg)
        return -limit_rad, limit_rad
    if any(value is not None for value in rad_values) and any(
        value is not None for value in deg_values
    ):
        parser.error("use either radian limits or degree limits, not both")
    if any(value is not None for value in deg_values):
        if any(value is None for value in deg_values):
            parser.error("both --min-position-deg and --max-position-deg are required")
        return math.radians(args.min_position_deg), math.radians(
            args.max_position_deg
        )
    if all(value is None for value in rad_values):
        return (
            END_EFFECTOR_ROTATION_CONFIG.min_position_rad,
            END_EFFECTOR_ROTATION_CONFIG.max_position_rad,
        )
    if any(value is None for value in rad_values):
        parser.error(
            "provide --limit-deg, degree min/max, or radian min/max limits"
        )
    return args.min_position_rad, args.max_position_rad


def _validate_cli(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    profile = FEETECH_MODEL_PROFILES[args.model]
    if (
        args.counts_per_turn is not None
        and args.counts_per_turn != profile.counts_per_turn
    ):
        parser.error(
            f"--counts-per-turn must be {profile.counts_per_turn} for {profile.model}"
        )
    position_requested = (
        args.position_rad is not None or args.position_deg is not None
    )
    if args.enable_torque and not position_requested:
        parser.error("--enable-torque is only valid with a position command")
    calibrated_operations = position_requested or args.read_position
    if calibrated_operations:
        (
            args._resolved_min_position_rad,
            args._resolved_max_position_rad,
        ) = _resolve_limits(args, parser)
        args._resolved_max_speed_raw = args.max_speed_raw
        args._resolved_position_rad = (
            math.radians(args.position_deg)
            if args.position_deg is not None
            else args.position_rad
        )
    if args.execute and not args.port:
        parser.error("--execute requires explicit --port")
    if not args.execute and args.enable_torque:
        parser.error("--enable-torque requires --execute")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_cli(args, parser)
    profile = FEETECH_MODEL_PROFILES[args.model]
    baudrate = args.baudrate or profile.default_baudrate
    position_requested = (
        args.position_rad is not None or args.position_deg is not None
    )
    config = _config(args) if position_requested or args.read_position else None
    validated_position_command: tuple[int, bytes] | None = None
    if position_requested:
        assert config is not None
        validated_position_command = build_position_payload(
            args._resolved_position_rad,
            args.speed_raw,
            config,
            move_time_raw=args.move_time_raw,
            acceleration_raw=args.acceleration_raw,
        )

    if not args.execute:
        preview: dict[str, object] = {
            "mode": "dry-run",
            "model": profile.model,
            "protocol": profile.protocol,
            "transport": profile.transport,
            "adapter_auto_direction": profile.adapter_auto_direction,
            "baudrate": baudrate,
            "counts_per_turn": profile.counts_per_turn,
            "servo_id": args.servo_id,
            "project_config": "END_EFFECTOR_ROTATION_CONFIG",
        }
        if args.ping:
            preview.update(
                operation="ping",
                packet_hex=build_instruction_packet(
                    args.servo_id, INSTRUCTION_PING
                ).hex(),
            )
        elif args.read_raw_position or args.read_position:
            preview.update(
                operation=(
                    "read-raw-position"
                    if args.read_raw_position
                    else "read-position"
                ),
                packet_hex=build_instruction_packet(
                    args.servo_id,
                    INSTRUCTION_READ,
                    bytes((profile.registers.present_position, 2)),
                ).hex(),
            )
        elif args.disable:
            preview.update(
                operation="disable",
                packet_hex=build_instruction_packet(
                    args.servo_id,
                    INSTRUCTION_WRITE,
                    bytes((profile.registers.torque_enable, 0)),
                ).hex(),
            )
        if position_requested:
            assert config is not None
            assert validated_position_command is not None
            target_raw, payload = validated_position_command
            packet = build_instruction_packet(
                args.servo_id,
                INSTRUCTION_WRITE,
                bytes((config.registers.goal_position,)) + payload,
            )
            preview.update(
                operation="position",
                target_position_deg=math.degrees(args._resolved_position_rad),
                limits_deg=[
                    math.degrees(args._resolved_min_position_rad),
                    math.degrees(args._resolved_max_position_rad),
                ],
                zero_raw=args.zero_raw,
                direction_sign=args.direction_sign,
                positive_direction=END_EFFECTOR_ROTATION_POSITIVE_DIRECTION,
                speed_raw=args.speed_raw,
                max_speed_raw=args._resolved_max_speed_raw,
                target_raw=target_raw,
                write_payload_hex=payload.hex(),
                packet_hex=packet.hex(),
            )
        print(json.dumps(preview, ensure_ascii=False, indent=2))
        return 0

    serial_config = FeetechSerialConfig(
        port=args.port,
        baudrate=baudrate,
        timeout=args.timeout,
        write_timeout=args.timeout,
    )
    with FeetechBus(serial_config) as bus:
        if args.ping:
            bus.ping(args.servo_id)
            print(json.dumps({"servo_id": args.servo_id, "ping": "ok"}, indent=2))
        elif args.read_raw_position:
            data = bus.read_registers(
                args.servo_id, profile.registers.present_position, 2
            )
            raw = int.from_bytes(data, "little")
            print(
                json.dumps(
                    {
                        "position_raw": raw,
                        "single_turn_deg": raw * 360.0 / profile.counts_per_turn,
                    },
                    indent=2,
                )
            )
        elif args.disable:
            bus.write_registers(
                args.servo_id,
                profile.registers.torque_enable,
                b"\x00",
                expect_status=args.expect_write_status,
            )
            print("torque disabled")
        elif args.read_position:
            assert config is not None
            axis = FeetechRotationAxis(bus, config)
            print(json.dumps({"position_rad": axis.read_position()}, indent=2))
        else:
            assert config is not None
            axis = FeetechRotationAxis(bus, config)
            if args.enable_torque:
                axis.enable_torque()
            target_raw = axis.command_position(
                args._resolved_position_rad,
                args.speed_raw,
                move_time_raw=args.move_time_raw,
                acceleration_raw=args.acceleration_raw,
            )
            print(json.dumps({"commanded_target_raw": target_raw}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
