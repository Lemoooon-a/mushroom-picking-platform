#!/usr/bin/env python3
"""Public manual motion debug interface.

Real motion requires explicit execution and risk confirmation.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import math
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[2]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from motion.authorization import RuntimeMode  # noqa: E402
from motion.unified_protocol import (  # noqa: E402
    AxisName,
    AxisTarget,
    MotionCommandStatus,
    MultiAxisTarget,
)
from scripts._motion_cli_common import (  # noqa: E402
    best_effort_stop_axes_once,
    create_configured_runtime,
    format_axis_descriptor,
    format_axis_state,
    format_command_result,
    initialize_read_only_rotary_positions,
    motion_state_blockers,
    positive_float,
    prepare_rotation_power,
)


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("value must be finite")
    return parsed


def run_axis_state(
    runtime: object,
    axis: AxisName,
    *,
    emit: Callable[[str], None] = print,
) -> bool:
    with runtime:
        initialize_read_only_rotary_positions(runtime, (axis,))
        descriptor = runtime.controller.describe_axis(axis)
        state = runtime.controller.get_state(axis)
        emit(format_axis_descriptor(descriptor))
        emit(format_axis_state(state))
    return True


def run_axis_move(
    runtime: object,
    target: AxisTarget,
    *,
    execute: bool,
    timeout_s: float | None,
    confirm_rotation_no_stop: bool = False,
    confirm_rotation_torque_enable: bool = False,
    emit: Callable[[str], None] = print,
) -> bool:
    """预检或通过统一 controller 提交一个绝对目标。"""

    submitted = False
    terminal_result_received = False
    with runtime:
        initialize_read_only_rotary_positions(runtime, (target.axis,))
        descriptor = runtime.controller.describe_axis(target.axis)
        state = runtime.controller.get_state(target.axis)
        emit(format_axis_descriptor(descriptor))
        emit(format_axis_state(state))
        runtime.controller.validate_positions(MultiAxisTarget((target,)))
        emit(
            f"planned absolute target: axis={target.axis.value} "
            f"position={target.position:.6f} {descriptor.position_unit} "
            f"velocity={target.velocity} acceleration={target.acceleration}"
        )
        blockers = motion_state_blockers(target.axis, state)
        if blockers:
            emit("MOVE PREFLIGHT REJECTED: " + "; ".join(blockers))
            return False
        if not execute:
            emit("READ_ONLY preview complete; no position, enable, home, or stop was sent")
            if target.axis is AxisName.ROTATION:
                emit(
                    "Rotation execution additionally requires no-stop risk acceptance "
                    "and explicit torque preparation confirmation"
                )
            return True

        if target.axis is AxisName.ROTATION:
            prepare_rotation_power(
                runtime,
                state,
                confirm_no_independent_stop=confirm_rotation_no_stop,
                confirm_torque_enable=confirm_rotation_torque_enable,
                emit=emit,
            )

        try:
            handle = runtime.controller.submit_absolute(target)
            submitted = True
            result = runtime.controller.wait(handle, timeout_s=timeout_s)
            terminal_result_received = True
        except BaseException:
            if submitted and not terminal_result_received:
                best_effort_stop_axes_once(runtime, (target.axis,), emit=emit)
            raise

        emit(format_command_result(result))
        return result.status is MotionCommandStatus.ARRIVED


def run_axis_stop(
    runtime: object,
    axis: AxisName,
    *,
    execute: bool,
    emit: Callable[[str], None] = print,
) -> bool:
    with runtime:
        descriptor = runtime.controller.describe_axis(axis)
        state = runtime.controller.get_state(axis)
        emit(format_axis_descriptor(descriptor))
        emit(format_axis_state(state))
        if axis in (AxisName.SLIDE, AxisName.Z):
            emit("planned operation: STM32 software/protocol stop; not disable or power cut")
        elif axis in (AxisName.SHOULDER, AxisName.ELBOW):
            emit("planned operation: MG4010 software stop (0x81); not disable")
        else:
            emit("Rotation has no verified independent stop; no command will be sent")
            return False
        if not execute:
            emit("READ_ONLY stop preview complete; no stop command was sent")
            return True
        result = runtime.controller.stop(axis)
        emit(format_command_result(result))
        return result.accepted


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect, move, or software-stop one upper-motion axis."
    )
    commands = parser.add_subparsers(dest="command", required=True)

    state = commands.add_parser("state", help="read descriptor and logical state")
    state.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)

    move = commands.add_parser("move", help="preview or submit one absolute target")
    move.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)
    move.add_argument("--position", type=_finite_float, required=True)
    move.add_argument("--velocity", type=positive_float)
    move.add_argument("--acceleration", type=positive_float)
    move.add_argument("--timeout", type=positive_float)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm-motion", action="store_true")
    move.add_argument("--allow-rotation-motion", action="store_true")
    move.add_argument("--confirm-rotation-no-stop", action="store_true")
    move.add_argument("--enable-rotation-torque", action="store_true")

    stop = commands.add_parser("stop", help="preview or send one software stop")
    stop.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)
    stop.add_argument("--execute", action="store_true")
    stop.add_argument("--confirm-stop", action="store_true")
    return parser


def _validate_flags(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.command == "move":
        if args.execute != args.confirm_motion:
            parser.error("real motion requires both --execute and --confirm-motion")
        rotation_flags = (
            args.allow_rotation_motion,
            args.confirm_rotation_no_stop,
            args.enable_rotation_torque,
        )
        is_rotation = args.axis == AxisName.ROTATION.value
        if is_rotation and args.execute and not all(rotation_flags):
            parser.error(
                "Rotation motion also requires --allow-rotation-motion, "
                "--confirm-rotation-no-stop, and --enable-rotation-torque"
            )
        if (not is_rotation or not args.execute) and any(rotation_flags):
            parser.error("Rotation risk flags are only valid for executed Rotation motion")
    elif args.command == "stop" and args.execute != args.confirm_stop:
        parser.error("real software stop requires both --execute and --confirm-stop")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate_flags(parser, args)
    axis = AxisName(args.axis)
    execute = bool(getattr(args, "execute", False))
    mode = RuntimeMode.MOTION if execute else RuntimeMode.READ_ONLY
    allow_rotation = bool(
        getattr(args, "allow_rotation_motion", False) and execute
    )
    try:
        runtime = create_configured_runtime(
            mode,
            allow_unverified_rotation_motion=allow_rotation,
        )
        if args.command == "state":
            succeeded = run_axis_state(runtime, axis)
        elif args.command == "move":
            succeeded = run_axis_move(
                runtime,
                AxisTarget(axis, args.position, args.velocity, args.acceleration),
                execute=execute,
                timeout_s=args.timeout,
                confirm_rotation_no_stop=args.confirm_rotation_no_stop,
                confirm_rotation_torque_enable=args.enable_rotation_torque,
            )
        else:
            succeeded = run_axis_stop(runtime, axis, execute=execute)
        return 0 if succeeded else 1
    except KeyboardInterrupt:
        print(
            "axis command interrupted; the CLI attempted at most one software stop "
            "for the submitted axis",
            file=sys.stderr,
        )
        return 130
    except Exception as exc:
        print(f"upper-motion axis command failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
