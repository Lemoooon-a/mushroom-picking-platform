#!/usr/bin/env python3
"""Public multi-axis point-to-point debug interface.

Not interpolated or strictly synchronized motion.
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
from motion.unified_controller import (  # noqa: E402
    MultiAxisSubmissionError,
    UnifiedMotionError,
)
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
    format_group_result,
    initialize_read_only_rotary_positions,
    motion_state_blockers,
    positive_float,
    prepare_rotation_power,
)


_AXIS_ARGUMENTS = {
    AxisName.SLIDE: "slide",
    AxisName.Z: "z",
    AxisName.SHOULDER: "shoulder",
    AxisName.ELBOW: "elbow",
    AxisName.ROTATION: "rotation",
}


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("value must be finite")
    return parsed


def run_multi_axis_test(
    runtime: object,
    target: MultiAxisTarget,
    *,
    execute: bool,
    timeout_s: float,
    max_linear_delta_mm: float | None = None,
    max_rotary_delta_deg: float | None = None,
    confirm_rotation_no_stop: bool = False,
    confirm_rotation_torque_enable: bool = False,
    emit: Callable[[str], None] = print,
) -> bool:
    """预检或提交用户明确指定的任意轴子集。"""

    if not isinstance(target, MultiAxisTarget):
        raise TypeError("target must be a MultiAxisTarget")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and greater than zero")
    for value, name in (
        (max_linear_delta_mm, "max_linear_delta_mm"),
        (max_rotary_delta_deg, "max_rotary_delta_deg"),
    ):
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError(f"{name} must be finite and greater than zero")

    axes = tuple(item.axis for item in target.targets)
    submitted = False
    terminal_result_received = False
    with runtime:
        initialize_read_only_rotary_positions(runtime, axes)
        descriptors = {
            axis: runtime.controller.describe_axis(axis) for axis in axes
        }
        states = runtime.controller.get_axis_states(axes)
        state_by_axis = {state.axis: state for state in states}
        if tuple(state_by_axis) != axes:
            raise ValueError("unified state query did not preserve the target axis order")

        try:
            runtime.controller.validate_positions(target)
        except UnifiedMotionError as exc:
            emit(f"MULTI-AXIS PREFLIGHT REJECTED: {exc}")
            return False

        blockers: list[str] = []
        emit("participating axes and planned targets:")
        for item in target.targets:
            descriptor = descriptors[item.axis]
            state = state_by_axis[item.axis]
            emit(f"  {format_axis_descriptor(descriptor)}")
            emit(f"  {format_axis_state(state)}")
            emit(
                f"  target={item.position:.6f} {descriptor.position_unit} "
                f"velocity={item.velocity} acceleration={item.acceleration}"
            )
            blockers.extend(
                f"{item.axis.value}: {reason}"
                for reason in motion_state_blockers(item.axis, state)
            )
            if state.current_position is not None:
                delta = abs(item.position - state.current_position)
                maximum_delta = (
                    max_linear_delta_mm
                    if item.axis in (AxisName.SLIDE, AxisName.Z)
                    else max_rotary_delta_deg
                )
                if maximum_delta is not None and delta > maximum_delta:
                    blockers.append(
                        f"{item.axis.value}: delta {delta:.6f} "
                        f"{descriptor.position_unit} exceeds invocation limit "
                        f"{maximum_delta:.6f} {descriptor.position_unit}"
                    )
        if blockers:
            emit("MULTI-AXIS PREFLIGHT REJECTED: " + "; ".join(blockers))
            return False
        if not execute:
            emit(
                "READ_ONLY preview complete; only explicitly listed axes participate, "
                "and no command was submitted"
            )
            return True

        if AxisName.ROTATION in state_by_axis:
            prepare_rotation_power(
                runtime,
                state_by_axis[AxisName.ROTATION],
                confirm_no_independent_stop=confirm_rotation_no_stop,
                confirm_torque_enable=confirm_rotation_torque_enable,
                emit=emit,
            )

        try:
            handle = runtime.controller.submit_positions(target)
            submitted = True
            result = runtime.controller.wait_group(handle, timeout_s=timeout_s)
            terminal_result_received = True
        except MultiAxisSubmissionError as exc:
            # submit_positions() already owns partial-submission peer stop.
            for line in format_group_result(exc.result):
                emit(line)
            emit("submission failed; controller already handled partial-submission stop")
            return False
        except BaseException:
            if submitted and not terminal_result_received:
                # One helper invocation; each participating stoppable axis is called once.
                best_effort_stop_axes_once(runtime, axes, emit=emit)
            raise

        for line in format_group_result(result):
            emit(line)
        if result.status is not MotionCommandStatus.ARRIVED:
            emit(
                "terminal group failure received; controller owns timeout/fault/peer "
                "best-effort stop, so the CLI will not repeat it"
            )
            return False
        return True


# 旧脚本和外部 import 的兼容函数名；实现仍只有一个。
run_five_axis_test = run_multi_axis_test


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview or submit a point-to-point target for any explicit axis subset. "
            "Submission is back-to-back, not interpolated or strictly synchronized."
        )
    )
    for axis, name in _AXIS_ARGUMENTS.items():
        parser.add_argument(f"--{name}", type=_finite_float, help=f"{axis.value} target")
        parser.add_argument(f"--{name}-velocity", type=positive_float)
        parser.add_argument(f"--{name}-acceleration", type=positive_float)
    parser.add_argument("--timeout", type=positive_float, default=10.0)
    parser.add_argument("--max-linear-delta-mm", type=positive_float)
    parser.add_argument("--max-rotary-delta-deg", type=positive_float)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-motion", action="store_true")
    parser.add_argument("--allow-rotation-motion", action="store_true")
    parser.add_argument("--confirm-rotation-no-stop", action="store_true")
    parser.add_argument("--enable-rotation-torque", action="store_true")
    return parser


def _target_from_args(args: argparse.Namespace) -> MultiAxisTarget:
    targets: list[AxisTarget] = []
    for axis, name in _AXIS_ARGUMENTS.items():
        position = getattr(args, name)
        velocity = getattr(args, f"{name}_velocity")
        acceleration = getattr(args, f"{name}_acceleration")
        if position is None:
            if velocity is not None or acceleration is not None:
                raise ValueError(f"--{name}-velocity/acceleration requires --{name}")
            continue
        targets.append(AxisTarget(axis, position, velocity, acceleration))
    if not targets:
        raise ValueError("at least one axis target is required")
    return MultiAxisTarget(tuple(targets))


def _validate_flags(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    target: MultiAxisTarget,
) -> None:
    if args.execute != args.confirm_motion:
        parser.error("real motion requires both --execute and --confirm-motion")
    rotation_flags = (
        args.allow_rotation_motion,
        args.confirm_rotation_no_stop,
        args.enable_rotation_torque,
    )
    includes_rotation = any(item.axis is AxisName.ROTATION for item in target.targets)
    if includes_rotation and args.execute and not all(rotation_flags):
        parser.error(
            "Rotation motion also requires --allow-rotation-motion, "
            "--confirm-rotation-no-stop, and --enable-rotation-torque"
        )
    if (not includes_rotation or not args.execute) and any(rotation_flags):
        parser.error("Rotation risk flags are only valid for executed Rotation motion")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        target = _target_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    _validate_flags(parser, args, target)
    mode = RuntimeMode.MOTION if args.execute else RuntimeMode.READ_ONLY
    try:
        runtime = create_configured_runtime(
            mode,
            allow_unverified_rotation_motion=(
                args.execute and args.allow_rotation_motion
            ),
        )
        print(
            "This is back-to-back point-to-point submission: no interpolation, "
            "simultaneous start, strict synchronization, or simultaneous arrival."
        )
        succeeded = run_multi_axis_test(
            runtime,
            target,
            execute=args.execute,
            timeout_s=args.timeout,
            max_linear_delta_mm=args.max_linear_delta_mm,
            max_rotary_delta_deg=args.max_rotary_delta_deg,
            confirm_rotation_no_stop=args.confirm_rotation_no_stop,
            confirm_rotation_torque_enable=args.enable_rotation_torque,
        )
        return 0 if succeeded else 1
    except KeyboardInterrupt:
        print(
            "multi-axis command interrupted; each participating stoppable axis was "
            "sent at most one CLI-owned software stop",
            file=sys.stderr,
        )
        return 130
    except Exception as exc:
        print(f"upper-motion multi-axis command failed: {exc}", file=sys.stderr)
        return 2


def legacy_main(argv: Sequence[str] | None = None) -> int:
    """把旧五轴参数无 subprocess 地映射到新轴子集 CLI。"""

    parser = argparse.ArgumentParser(
        description="Deprecated five-axis compatibility parser",
    )
    for name in ("slide", "z", "shoulder", "elbow", "rotation"):
        suffix = "mm" if name in ("slide", "z") else "deg"
        parser.add_argument(f"--{name}-{suffix}", type=_finite_float, required=True)
    parser.add_argument("--slide-speed-mm-s", type=positive_float)
    parser.add_argument("--slide-accel-mm-s2", type=positive_float)
    parser.add_argument("--z-speed-mm-s", type=positive_float)
    parser.add_argument("--z-accel-mm-s2", type=positive_float)
    parser.add_argument("--shoulder-speed-deg-s", type=positive_float)
    parser.add_argument("--elbow-speed-deg-s", type=positive_float)
    parser.add_argument("--timeout", type=positive_float, required=True)
    parser.add_argument("--max-linear-delta-mm", type=positive_float, required=True)
    parser.add_argument("--max-rotary-delta-deg", type=positive_float, required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-five-axis-motion", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    parser.add_argument("--accept-nonstrict-synchronization", action="store_true")
    parser.add_argument("--accept-unverified-rotation-stop", action="store_true")
    parser.add_argument("--enable-rotation-torque", action="store_true")
    args = parser.parse_args(argv)
    legacy_confirmations = (
        args.confirm_five_axis_motion,
        args.confirm_emergency_stop_ready,
        args.accept_nonstrict_synchronization,
        args.accept_unverified_rotation_stop,
        args.enable_rotation_torque,
    )
    if args.execute and not all(legacy_confirmations):
        parser.error("legacy --execute still requires all five legacy confirmations")
    if not args.execute and any(legacy_confirmations):
        parser.error("legacy motion confirmations require --execute")

    translated = [
        "--slide", str(args.slide_mm),
        "--z", str(args.z_mm),
        "--shoulder", str(args.shoulder_deg),
        "--elbow", str(args.elbow_deg),
        "--rotation", str(args.rotation_deg),
        "--timeout", str(args.timeout),
        "--max-linear-delta-mm", str(args.max_linear_delta_mm),
        "--max-rotary-delta-deg", str(args.max_rotary_delta_deg),
    ]
    optional = (
        ("--slide-velocity", args.slide_speed_mm_s),
        ("--slide-acceleration", args.slide_accel_mm_s2),
        ("--z-velocity", args.z_speed_mm_s),
        ("--z-acceleration", args.z_accel_mm_s2),
        ("--shoulder-velocity", args.shoulder_speed_deg_s),
        ("--elbow-velocity", args.elbow_speed_deg_s),
    )
    for flag, value in optional:
        if value is not None:
            translated.extend((flag, str(value)))
    if args.execute:
        translated.extend(
            (
                "--execute",
                "--confirm-motion",
                "--allow-rotation-motion",
                "--confirm-rotation-no-stop",
                "--enable-rotation-torque",
            )
        )
    return main(translated)


if __name__ == "__main__":
    raise SystemExit(main())
