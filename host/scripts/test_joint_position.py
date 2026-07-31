#!/usr/bin/env python3
"""MG4010 有限行程关节位置命令的只读预览与显式人工测试工具。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
import math
from pathlib import Path
import sys

import can


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.can_bus import CanMotorBus  # noqa: E402
from config.joints import JOINT_CONFIGS  # noqa: E402
from drivers.mg4010_driver import MG4010Driver  # noqa: E402
from drivers.mg4010_protocol import (  # noqa: E402
    MotorError,
    build_position_command_2,
    build_request_id,
)
from robot.joint import (  # noqa: E402
    CanRotaryJoint,
    JointConfig,
    JointError,
    JointState,
    joint_velocity_to_motor_speed_deg_s,
    resolve_output_angle_to_joint_position,
    wrap_360,
)


LOGGER = logging.getLogger("test_joint_position")


@dataclass(frozen=True)
class PositionCommandPlan:
    """由当前 0x94/0x92 快照计算出的 A4 预览。"""

    circle_angle_raw: int
    motor_cycle_deg: float
    output_abs_deg: float
    current_joint_rad: float
    target_joint_rad: float
    delta_joint_rad: float
    motor_delta_deg: float
    current_motor_multi_turn_deg: float
    target_motor_multi_turn_deg: float
    max_motor_speed_deg_s: float
    angle_control_raw: int
    max_speed_raw: int
    request_id: int
    payload: bytes


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview or explicitly submit one calibrated MG4010 joint position "
            "command. Motion is disabled unless --enable-motion is present."
        )
    )
    parser.add_argument(
        "--joint",
        choices=tuple(JOINT_CONFIGS),
        help="use a complete named config from config/joints.py",
    )
    parser.add_argument("--motor-id", type=int, choices=range(1, 33))
    parser.add_argument("--target-rad", type=_finite_float, required=True)
    parser.add_argument("--velocity-rad-s", type=_positive_float, required=True)
    parser.add_argument("--gear-ratio", type=_positive_float)
    parser.add_argument(
        "--encoder-zero-output-deg", type=_finite_float
    )
    parser.add_argument("--direction-sign", type=int, choices=(-1, 1))
    parser.add_argument("--min-position-rad", type=_finite_float)
    parser.add_argument("--max-position-rad", type=_finite_float)
    parser.add_argument(
        "--max-velocity-rad-s", type=_positive_float
    )
    parser.add_argument(
        "--position-tolerance-rad", type=_positive_float
    )
    parser.add_argument(
        "--moving-velocity-threshold-rad-s",
        type=_positive_float,
    )
    parser.add_argument("--interface", choices=("gs_usb", "socketcan"))
    parser.add_argument("--channel")
    parser.add_argument("--bitrate", type=_positive_int)
    parser.add_argument("--allow-same-id-response", action="store_true")
    parser.add_argument("--raw", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "offline preview; requires --current-circle-angle-raw and "
            "--current-multi-turn-deg and never opens CAN"
        ),
    )
    parser.add_argument(
        "--current-circle-angle-raw",
        type=_non_negative_int,
        help="offline 0x94 raw value used only with --dry-run",
    )
    parser.add_argument(
        "--current-multi-turn-deg",
        type=_finite_float,
        help="offline current 0x92 motor degrees used only with --dry-run",
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="explicitly permit one A4 submission; ignored when --dry-run is set",
    )
    return parser


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def _positive_float(value: str) -> float:
    parsed = _finite_float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def _non_negative_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def _raw_printer(direction: str, message: can.Message) -> None:
    data = " ".join(f"{byte:02X}" for byte in message.data)
    print(f"{direction} 0x{message.arbitration_id:03X} [{message.dlc}] {data}")


def _motion_frame_printer(show_all_frames: bool):
    """运动开启时始终显示真正提交的 A4/81 帧。"""

    def print_selected(direction: str, message: can.Message) -> None:
        is_final_motion_tx = (
            direction == "TX"
            and bool(message.data)
            and message.data[0] in (0xA4, 0x81)
        )
        if is_final_motion_tx:
            data = " ".join(f"{byte:02X}" for byte in message.data)
            print(
                f"FINAL-MOTION-TX 0x{message.arbitration_id:03X} "
                f"[{message.dlc}] {data}"
            )
        elif show_all_frames:
            _raw_printer(direction, message)

    return print_selected


def config_from_args(args: argparse.Namespace) -> JointConfig:
    manual_fields = (
        "motor_id",
        "gear_ratio",
        "encoder_zero_output_deg",
        "direction_sign",
        "min_position_rad",
        "max_position_rad",
        "max_velocity_rad_s",
        "position_tolerance_rad",
        "moving_velocity_threshold_rad_s",
    )
    if args.joint is not None:
        supplied = [name for name in manual_fields if getattr(args, name) is not None]
        if supplied:
            rendered = ", ".join(f"--{name.replace('_', '-')}" for name in supplied)
            raise ValueError(
                f"--joint cannot be combined with manual config options: {rendered}"
            )
        return JOINT_CONFIGS[args.joint]

    required = (
        "motor_id",
        "encoder_zero_output_deg",
        "direction_sign",
        "min_position_rad",
        "max_position_rad",
    )
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        rendered = ", ".join(f"--{name.replace('_', '-')}" for name in missing)
        raise ValueError(f"use --joint or provide manual config options: {rendered}")

    gear_ratio = args.gear_ratio if args.gear_ratio is not None else 36.0
    return JointConfig(
        name=f"motor_{args.motor_id}_joint",
        motor_id=args.motor_id,
        gear_ratio=gear_ratio,
        direction_sign=args.direction_sign,
        encoder_zero_output_deg=args.encoder_zero_output_deg,
        min_position_rad=args.min_position_rad,
        max_position_rad=args.max_position_rad,
        max_velocity_rad_s=(
            args.max_velocity_rad_s
            if args.max_velocity_rad_s is not None
            else math.radians(0xFFFF / gear_ratio)
        ),
        position_tolerance_rad=(
            args.position_tolerance_rad
            if args.position_tolerance_rad is not None
            else math.radians(0.1)
        ),
        moving_velocity_threshold_rad_s=(
            args.moving_velocity_threshold_rad_s
            if args.moving_velocity_threshold_rad_s is not None
            else math.radians(0.05)
        ),
    )


def build_position_plan(
    *,
    circle_angle_raw: int,
    current_motor_multi_turn_deg: float,
    target_joint_rad: float,
    velocity_rad_s: float,
    config: JointConfig,
) -> PositionCommandPlan:
    """纯计算生成 A4 预览；不访问 CAN。"""

    cycle_raw_limit = round(36_000 * config.gear_ratio)
    if not 0 <= circle_angle_raw < cycle_raw_limit:
        raise ValueError(
            f"circle_angle_raw {circle_angle_raw} is outside [0, "
            f"{cycle_raw_limit}) for gear ratio {config.gear_ratio:g}"
        )
    motor_cycle_deg = circle_angle_raw / 100.0
    output_abs_deg = wrap_360(motor_cycle_deg / config.gear_ratio)
    current_joint_rad = resolve_output_angle_to_joint_position(
        output_abs_deg, config
    )
    if not math.isfinite(target_joint_rad) or not (
        config.min_position_rad <= target_joint_rad <= config.max_position_rad
    ):
        raise ValueError(
            f"target {target_joint_rad!r} rad is outside calibrated range "
            f"[{config.min_position_rad}, {config.max_position_rad}]"
        )
    delta_joint_rad = target_joint_rad - current_joint_rad
    motor_delta_deg = (
        config.direction_sign
        * math.degrees(delta_joint_rad)
        * config.gear_ratio
    )
    target_motor_multi_turn_deg = current_motor_multi_turn_deg + motor_delta_deg
    max_motor_speed_deg_s = joint_velocity_to_motor_speed_deg_s(
        velocity_rad_s, config
    )
    payload = build_position_command_2(
        target_motor_deg=target_motor_multi_turn_deg,
        max_motor_speed_deg_s=max_motor_speed_deg_s,
    )
    return PositionCommandPlan(
        circle_angle_raw=circle_angle_raw,
        motor_cycle_deg=motor_cycle_deg,
        output_abs_deg=output_abs_deg,
        current_joint_rad=current_joint_rad,
        target_joint_rad=target_joint_rad,
        delta_joint_rad=delta_joint_rad,
        motor_delta_deg=motor_delta_deg,
        current_motor_multi_turn_deg=current_motor_multi_turn_deg,
        target_motor_multi_turn_deg=target_motor_multi_turn_deg,
        max_motor_speed_deg_s=max_motor_speed_deg_s,
        angle_control_raw=round(target_motor_multi_turn_deg * 100),
        max_speed_raw=round(max_motor_speed_deg_s),
        request_id=build_request_id(config.motor_id),
        payload=payload,
    )


def build_plan_from_state(
    state: JointState,
    *,
    target_joint_rad: float,
    velocity_rad_s: float,
    config: JointConfig,
) -> PositionCommandPlan:
    if state.motor_multi_turn_deg is None:
        raise ValueError("live state does not contain a current 0x92 coordinate")
    return build_position_plan(
        circle_angle_raw=state.circle_angle_raw,
        current_motor_multi_turn_deg=state.motor_multi_turn_deg,
        target_joint_rad=target_joint_rad,
        velocity_rad_s=velocity_rad_s,
        config=config,
    )


def print_plan(plan: PositionCommandPlan, motor_id: int) -> None:
    print(f"CAN motor ID                 : {motor_id}")
    print(f"circle_angle_raw             : {plan.circle_angle_raw}")
    print(f"motor_cycle_deg              : {plan.motor_cycle_deg:.6f}")
    print(f"output_abs_deg               : {plan.output_abs_deg:.6f}")
    print(f"current joint position       : {plan.current_joint_rad:.9f} rad")
    print(f"target joint position        : {plan.target_joint_rad:.9f} rad")
    print(f"delta_joint_rad              : {plan.delta_joint_rad:+.9f}")
    print(f"motor_delta_deg              : {plan.motor_delta_deg:+.6f}")
    print(
        "current 0x92 motor position : "
        f"{plan.current_motor_multi_turn_deg:.6f} deg"
    )
    print(
        "target A4 motor position    : "
        f"{plan.target_motor_multi_turn_deg:.6f} deg"
    )
    print(f"max_motor_speed_deg_s        : {plan.max_motor_speed_deg_s:.6f}")
    print(f"angle_control_raw            : {plan.angle_control_raw}")
    print(f"max_speed_raw                : {plan.max_speed_raw}")
    print(f"request CAN ID               : 0x{plan.request_id:03X}")
    print(f"request DATA                 : {' '.join(f'{b:02X}' for b in plan.payload)}")


def _offline_dry_run(args: argparse.Namespace, config: JointConfig) -> int:
    if args.current_circle_angle_raw is None or args.current_multi_turn_deg is None:
        raise ValueError(
            "--dry-run requires --current-circle-angle-raw and "
            "--current-multi-turn-deg so no CAN device needs to be opened"
        )
    print("DRY RUN - NO MOTOR COMMAND WILL BE SENT")
    plan = build_position_plan(
        circle_angle_raw=args.current_circle_angle_raw,
        current_motor_multi_turn_deg=args.current_multi_turn_deg,
        target_joint_rad=args.target_rad,
        velocity_rad_s=args.velocity_rad_s,
        config=config,
    )
    print_plan(plan, config.motor_id)
    return 0


def _live_run(args: argparse.Namespace, config: JointConfig) -> int:
    motion_enabled = args.enable_motion and not args.dry_run
    print("MOTION ENABLED" if motion_enabled else "DRY RUN - NO MOTOR COMMAND WILL BE SENT")
    with CanMotorBus(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
        allow_same_id_response=args.allow_same_id_response,
        raw_frame_callback=(
            _motion_frame_printer(args.raw)
            if motion_enabled
            else (_raw_printer if args.raw else None)
        ),
    ) as bus:
        driver = MG4010Driver(bus, config.motor_id)
        joint = CanRotaryJoint(driver, config)
        joint.initialize()
        state = joint.get_state()
        plan = build_plan_from_state(
            state,
            target_joint_rad=args.target_rad,
            velocity_rad_s=args.velocity_rad_s,
            config=config,
        )
        print_plan(plan, config.motor_id)
        if not motion_enabled:
            return 0
        print(
            "Submitting the calibrated target with 0xA4; the final actual frame "
            "is printed immediately before transmission"
        )
        joint.command_position(args.target_rad, args.velocity_rad_s)
        print("Position command accepted by motor; mechanical arrival is not implied")
        return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        config = config_from_args(args)
        if args.dry_run:
            return _offline_dry_run(args, config)
        return _live_run(args, config)
    except (JointError, MotorError, can.CanError, OSError, ValueError) as exc:
        LOGGER.error("joint position test failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
