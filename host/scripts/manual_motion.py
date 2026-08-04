#!/usr/bin/env python3
"""Unified manual upper-motion CLI.

All normal axis commands go through one runtime and UnifiedMotionController.
Real motion, homing, and software stop require explicit confirmations.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import math
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from config.frame_transforms import (  # noqa: E402
    FrameTransformsDocument,
    load_frame_transforms_document,
)
from geometry.rigid_transform import RigidTransform  # noqa: E402
from kinematics.base_frame_solver import (  # noqa: E402
    BaseFrameFiveAxisSolver,
    FiveAxisNoSolutionError,
)
from kinematics.five_axis import (  # noqa: E402
    FiveAxisKinematics,
    load_local_five_axis_kinematics,
)
from kinematics.frame_chain import RobotAxisState  # noqa: E402
from motion.authorization import RuntimeMode  # noqa: E402
from motion.unified_controller import MultiAxisSubmissionError, UnifiedMotionError  # noqa: E402
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
    format_group_result,
    initialize_read_only_rotary_positions,
    motion_state_blockers,
    positive_float,
    prepare_rotation_power,
)


_AXIS_ORDER = tuple(AxisName)
_LINEAR_AXES = (AxisName.SLIDE, AxisName.Z)
_STOPPABLE_AXES = (AxisName.SLIDE, AxisName.Z, AxisName.SHOULDER, AxisName.ELBOW)
_HOME_TIMEOUTS_S = {AxisName.SLIDE: 15.0, AxisName.Z: 60.0}
_DEFAULT_FRAME_CONFIG = HOST_ROOT / "config" / "frame_transforms.local.json"


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def run_inspect(runtime: object, *, emit: Callable[[str], None] = print) -> None:
    """Read version, descriptors, and all five logical states."""

    with runtime:
        version = runtime.stm32_client.version()
        emit(
            "stm32: "
            f"protocol={version.protocol_version} firmware={version.firmware_version}"
        )
        initialize_read_only_rotary_positions(runtime, _AXIS_ORDER)
        descriptors = runtime.controller.list_axes()
        states = runtime.controller.get_axis_states(_AXIS_ORDER)
        emit("unified axis descriptors:")
        for descriptor in descriptors:
            emit(f"  {format_axis_descriptor(descriptor)}")
        emit("unified axis states:")
        for state in states:
            emit(f"  {format_axis_state(state)}")


def run_state(
    runtime: object,
    axis: AxisName,
    *,
    emit: Callable[[str], None] = print,
) -> bool:
    with runtime:
        initialize_read_only_rotary_positions(runtime, (axis,))
        emit(format_axis_descriptor(runtime.controller.describe_axis(axis)))
        emit(format_axis_state(runtime.controller.get_state(axis)))
    return True


def run_plan_base(
    runtime: object,
    base_T_tool_target: RigidTransform,
    *,
    fixed_slide_mm: float | None,
    allow_unvalidated_frame_transform: bool,
    frame_config: Path = _DEFAULT_FRAME_CONFIG,
    frame_document: FrameTransformsDocument | None = None,
    five_axis_kinematics: FiveAxisKinematics | None = None,
    emit: Callable[[str], None] = print,
) -> bool:
    """只读读取五轴状态并把 Base TCP 目标预览为 ``MultiAxisTarget``。"""

    if not isinstance(base_T_tool_target, RigidTransform):
        raise TypeError("base_T_tool_target must be RigidTransform")
    if not isinstance(allow_unvalidated_frame_transform, bool):
        raise TypeError("allow_unvalidated_frame_transform must be bool")
    emit("Base target:")
    _emit_transform(base_T_tool_target, emit=emit)

    with runtime:
        initialize_read_only_rotary_positions(runtime, _AXIS_ORDER)
        descriptors = runtime.controller.list_axes()
        states = runtime.controller.get_axis_states(_AXIS_ORDER)
        current_state = _robot_axis_state(states)
        document = (
            frame_document
            if frame_document is not None
            else load_frame_transforms_document(frame_config)
        )
        model = (
            five_axis_kinematics
            if five_axis_kinematics is not None
            else load_local_five_axis_kinematics()
        )
        if not isinstance(document, FrameTransformsDocument):
            raise TypeError("frame_document must be FrameTransformsDocument")
        if not isinstance(model, FiveAxisKinematics):
            raise TypeError("plan-base requires the built-in FiveAxisKinematics model")
        base_transform_validated = document.metadata.get("validated") is True
        emit("Loaded base_T_slide_zero:")
        _emit_transform(document.transforms.base_T_slide_zero, emit=emit)
        emit(f"  validated: {base_transform_validated}")
        descriptor_by_axis = {descriptor.name: descriptor for descriptor in descriptors}
        solver = BaseFrameFiveAxisSolver(
            five_axis_kinematics=model,
            base_T_slide_zero=document.transforms.base_T_slide_zero,
            axis_descriptors=descriptor_by_axis,
            base_transform_validated=base_transform_validated,
            allow_unvalidated_base_transform=allow_unvalidated_frame_transform,
        )
        slide_zero_target = solver.transform_base_target_to_slide_zero(
            base_T_tool_target
        )
        emit("Converted Slide-zero target:")
        _emit_transform(slide_zero_target, emit=emit)
        emit("Current axis state:")
        for state in states:
            emit(f"  {format_axis_state(state)}")
        candidates = solver.solve_base_target_candidates(
            base_T_tool_target=base_T_tool_target,
            current_state=current_state,
            fixed_slide_mm=fixed_slide_mm,
        )
        selected = candidates[0]
        target = solver.solution_to_multi_axis_target(selected)
        runtime.controller.validate_positions(target)

    emit(f"Candidate count: {len(candidates)}")
    emit("Selected solution:")
    emit(f"  slide: {selected.slide_mm:.9f} mm")
    emit(f"  z: {selected.z_mm:.9f} mm")
    emit(f"  shoulder: {selected.shoulder_deg:.9f} deg")
    emit(f"  elbow: {selected.elbow_deg:.9f} deg")
    emit(f"  rotation: {selected.rotation_deg:.9f} deg")
    emit(f"Branch: {selected.branch}")
    emit(f"Slide selection: {selected.slide_selection_reason}")
    emit(f"Score: {selected.score:.9f}")
    emit(f"Position residual: {selected.position_residual_mm:.9g} mm")
    emit(f"Yaw residual: {selected.yaw_residual_deg:.9g} deg")
    emit("Limit margins:")
    for axis, margin in selected.limit_margins:
        unit = "mm" if axis in _LINEAR_AXES else "deg"
        emit(f"  {axis.value}: {margin:.9f} {unit}")
    emit("Generated MultiAxisTarget (actuator-space logical targets; no Cartesian frame):")
    for item in target.targets:
        unit = "mm" if item.axis in _LINEAR_AXES else "deg"
        emit(
            f"  axis={item.axis.value} position={item.position:.9f} {unit} "
            f"velocity={item.velocity} acceleration={item.acceleration}"
        )
    emit("READ_ONLY plan-base preview complete; no command was submitted")
    return True


def run_move(
    runtime: object,
    target: AxisTarget,
    *,
    execute: bool,
    timeout_s: float | None,
    confirm_rotation_no_stop: bool = False,
    confirm_rotation_torque_enable: bool = False,
    emit: Callable[[str], None] = print,
) -> bool:
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
            emit("READ_ONLY preview complete; no control command was sent")
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


def run_move_group(
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
        descriptors = {axis: runtime.controller.describe_axis(axis) for axis in axes}
        states = runtime.controller.get_axis_states(axes)
        state_by_axis = {state.axis: state for state in states}
        if tuple(state_by_axis) != axes:
            raise ValueError("unified state query did not preserve target axis order")
        try:
            runtime.controller.validate_positions(target)
        except UnifiedMotionError as exc:
            emit(f"MOVE-GROUP PREFLIGHT REJECTED: {exc}")
            return False

        blockers: list[str] = []
        emit("back-to-back point-to-point preview; not interpolation or strict sync")
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
                maximum = (
                    max_linear_delta_mm
                    if item.axis in _LINEAR_AXES
                    else max_rotary_delta_deg
                )
                delta = abs(item.position - state.current_position)
                if maximum is not None and delta > maximum:
                    blockers.append(
                        f"{item.axis.value}: delta {delta:.6f} exceeds "
                        f"invocation limit {maximum:.6f} {descriptor.position_unit}"
                    )
        if blockers:
            emit("MOVE-GROUP PREFLIGHT REJECTED: " + "; ".join(blockers))
            return False
        if not execute:
            emit("READ_ONLY preview complete; only explicitly listed axes participate")
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
            for line in format_group_result(exc.result):
                emit(line)
            emit("controller already handled partial-submission peer stop")
            return False
        except BaseException:
            if submitted and not terminal_result_received:
                best_effort_stop_axes_once(runtime, axes, emit=emit)
            raise
        for line in format_group_result(result):
            emit(line)
        if result.status is not MotionCommandStatus.ARRIVED:
            emit("terminal group failure received; CLI will not repeat controller stop")
            return False
        return True


def run_home(
    runtime: object,
    axis: AxisName,
    *,
    execute: bool,
    timeout_s: float,
    emit: Callable[[str], None] = print,
) -> bool:
    if axis not in _LINEAR_AXES:
        raise ValueError("reference home only supports slide or z")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and greater than zero")
    terminal_result_received = False
    with runtime:
        before = runtime.controller.get_state(axis)
        emit(f"before: {format_axis_state(before)}")
        emit(f"planned operation: mechanical reference home axis={axis.value}")
        if not execute:
            emit("READ_ONLY preflight complete; no home or stop command was sent")
            return True
        blockers: list[str] = []
        if not before.connected:
            blockers.append("axis is not connected")
        if before.busy is not False:
            blockers.append("busy is not confirmed false")
        if before.faulted and before.fault_code != 2:
            blockers.append(f"blocking fault_code={before.fault_code!r}")
        if blockers:
            emit("HOME PREFLIGHT REJECTED: " + "; ".join(blockers))
            return False
        try:
            result = runtime.controller.home_reference(axis, timeout_s=timeout_s)
            terminal_result_received = True
            after = runtime.controller.get_state(axis)
        except BaseException:
            if not terminal_result_received:
                best_effort_stop_axes_once(runtime, (axis,), emit=emit)
            raise
        emit(f"home result: {format_command_result(result)}")
        emit(f"after: {format_axis_state(after)}")
        succeeded = (
            result.status is MotionCommandStatus.ARRIVED
            and after.homed is True
            and after.position_valid is True
            and after.busy is False
            and not after.faulted
        )
        if not succeeded:
            emit("terminal home result not verified; CLI will not repeat controller stop")
        return succeeded


def run_stop(
    runtime: object,
    axis: AxisName,
    *,
    execute: bool,
    emit: Callable[[str], None] = print,
) -> bool:
    with runtime:
        emit(format_axis_descriptor(runtime.controller.describe_axis(axis)))
        emit(format_axis_state(runtime.controller.get_state(axis)))
        if axis in _LINEAR_AXES:
            emit("planned operation: STM32 software/protocol stop")
        elif axis in (AxisName.SHOULDER, AxisName.ELBOW):
            emit("planned operation: MG4010 software stop (0x81)")
        else:
            emit("Rotation has no verified independent stop; unsupported")
            return False
        if not execute:
            emit("READ_ONLY stop preview complete; no command was sent")
            return True
        result = runtime.controller.stop(axis)
        emit(format_command_result(result))
        return result.accepted


def _add_rotation_flags(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--allow-rotation-motion", action="store_true")
    parser.add_argument("--confirm-rotation-no-stop", action="store_true")
    parser.add_argument("--enable-rotation-torque", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified manual upper-motion control")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("inspect", help="read all five axes and STM32 version")

    state = commands.add_parser("state", help="read one logical axis")
    state.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)

    plan_base = commands.add_parser(
        "plan-base",
        help="read-only Base TCP target to five-axis target preview",
    )
    plan_base.add_argument("--tcp-x-mm", type=finite_float, required=True)
    plan_base.add_argument("--tcp-y-mm", type=finite_float, required=True)
    plan_base.add_argument("--tcp-z-mm", type=finite_float, required=True)
    plan_base.add_argument("--tcp-yaw-deg", type=finite_float, required=True)
    plan_base.add_argument("--slide-mm", type=finite_float)
    plan_base.add_argument(
        "--allow-unvalidated-frame-transform",
        action="store_true",
    )

    move = commands.add_parser("move", help="preview or move one absolute axis target")
    move.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)
    move.add_argument("--position", type=finite_float, required=True)
    move.add_argument("--velocity", type=positive_float)
    move.add_argument("--acceleration", type=positive_float)
    move.add_argument("--timeout", type=positive_float)
    move.add_argument("--execute", action="store_true")
    move.add_argument("--confirm-motion", action="store_true")
    _add_rotation_flags(move)

    group = commands.add_parser("move-group", help="preview or move an explicit axis subset")
    for axis in _AXIS_ORDER:
        group.add_argument(f"--{axis.value}", type=finite_float)
        group.add_argument(f"--{axis.value}-velocity", type=positive_float)
        group.add_argument(f"--{axis.value}-acceleration", type=positive_float)
    group.add_argument("--timeout", type=positive_float, default=10.0)
    group.add_argument("--max-linear-delta-mm", type=positive_float)
    group.add_argument("--max-rotary-delta-deg", type=positive_float)
    group.add_argument("--execute", action="store_true")
    group.add_argument("--confirm-motion", action="store_true")
    _add_rotation_flags(group)

    home = commands.add_parser("home", help="preview or reference-home Slide/Z")
    home.add_argument("--axis", choices=("slide", "z"), required=True)
    home.add_argument("--timeout", type=positive_float)
    home.add_argument("--execute", action="store_true")
    home.add_argument("--confirm-home-motion", action="store_true")

    stop = commands.add_parser("stop", help="preview or send one software/protocol stop")
    stop.add_argument("--axis", choices=tuple(axis.value for axis in AxisName), required=True)
    stop.add_argument("--execute", action="store_true")
    stop.add_argument("--confirm-stop", action="store_true")
    return parser


def _group_target(args: argparse.Namespace, parser: argparse.ArgumentParser) -> MultiAxisTarget:
    targets: list[AxisTarget] = []
    for axis in _AXIS_ORDER:
        position = getattr(args, axis.value)
        velocity = getattr(args, f"{axis.value}_velocity")
        acceleration = getattr(args, f"{axis.value}_acceleration")
        if position is None:
            if velocity is not None or acceleration is not None:
                parser.error(f"--{axis.value}-velocity/acceleration requires --{axis.value}")
            continue
        targets.append(AxisTarget(axis, position, velocity, acceleration))
    if not targets:
        parser.error("move-group requires at least one axis target")
    return MultiAxisTarget(tuple(targets))


def _validate_confirmations(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
    axes: tuple[AxisName, ...],
) -> None:
    if args.command in ("move", "move-group"):
        if args.execute != args.confirm_motion:
            parser.error("real motion requires both --execute and --confirm-motion")
        rotation_flags = (
            args.allow_rotation_motion,
            args.confirm_rotation_no_stop,
            args.enable_rotation_torque,
        )
        includes_rotation = AxisName.ROTATION in axes
        if args.execute and includes_rotation and not all(rotation_flags):
            parser.error("Rotation motion requires all three Rotation risk confirmations")
        if (not args.execute or not includes_rotation) and any(rotation_flags):
            parser.error("Rotation flags are only valid for executed Rotation motion")
    elif args.command == "home" and args.execute != args.confirm_home_motion:
        parser.error("real home requires --execute and --confirm-home-motion")
    elif args.command == "stop" and args.execute != args.confirm_stop:
        parser.error("real stop requires --execute and --confirm-stop")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    target: AxisTarget | MultiAxisTarget | None = None
    base_target: RigidTransform | None = None
    axes: tuple[AxisName, ...] = ()
    if args.command == "move":
        axis = AxisName(args.axis)
        target = AxisTarget(axis, args.position, args.velocity, args.acceleration)
        axes = (axis,)
    elif args.command == "move-group":
        target = _group_target(args, parser)
        axes = tuple(item.axis for item in target.targets)
    elif args.command == "plan-base":
        base_target = RigidTransform.from_xyz_yaw_deg(
            x_mm=args.tcp_x_mm,
            y_mm=args.tcp_y_mm,
            z_mm=args.tcp_z_mm,
            yaw_deg=args.tcp_yaw_deg,
        )
    elif args.command in ("state", "home", "stop"):
        axes = (AxisName(args.axis),)
    _validate_confirmations(parser, args, axes)

    execute = bool(getattr(args, "execute", False))
    mode = RuntimeMode.MOTION if execute else RuntimeMode.READ_ONLY
    allow_rotation = bool(getattr(args, "allow_rotation_motion", False) and execute)
    try:
        runtime = create_configured_runtime(
            mode,
            allow_unverified_rotation_motion=allow_rotation,
        )
        if args.command == "inspect":
            run_inspect(runtime)
            return 0
        if args.command == "plan-base":
            assert base_target is not None
            succeeded = run_plan_base(
                runtime,
                base_target,
                fixed_slide_mm=args.slide_mm,
                allow_unvalidated_frame_transform=(
                    args.allow_unvalidated_frame_transform
                ),
            )
        elif args.command == "state":
            succeeded = run_state(runtime, axes[0])
        elif args.command == "move":
            assert isinstance(target, AxisTarget)
            succeeded = run_move(
                runtime,
                target,
                execute=execute,
                timeout_s=args.timeout,
                confirm_rotation_no_stop=args.confirm_rotation_no_stop,
                confirm_rotation_torque_enable=args.enable_rotation_torque,
            )
        elif args.command == "move-group":
            assert isinstance(target, MultiAxisTarget)
            succeeded = run_move_group(
                runtime,
                target,
                execute=execute,
                timeout_s=args.timeout,
                max_linear_delta_mm=args.max_linear_delta_mm,
                max_rotary_delta_deg=args.max_rotary_delta_deg,
                confirm_rotation_no_stop=args.confirm_rotation_no_stop,
                confirm_rotation_torque_enable=args.enable_rotation_torque,
            )
        elif args.command == "home":
            timeout_s = args.timeout or _HOME_TIMEOUTS_S[axes[0]]
            succeeded = run_home(runtime, axes[0], execute=execute, timeout_s=timeout_s)
        else:
            succeeded = run_stop(runtime, axes[0], execute=execute)
        return 0 if succeeded else 1
    except FiveAxisNoSolutionError as exc:
        print(
            f"plan-base no solution: {exc}; stage={exc.stage}; "
            f"counts={exc.stage_counts}",
            file=sys.stderr,
        )
        return 1
    except KeyboardInterrupt:
        print("manual motion interrupted; CLI-owned stops were attempted at most once", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"manual motion failed: {exc}", file=sys.stderr)
        return 2


def _robot_axis_state(states: tuple[object, ...]) -> RobotAxisState:
    if not isinstance(states, tuple):
        raise ValueError("axis state query must return a tuple")
    state_by_axis = {getattr(state, "axis", None): state for state in states}
    if set(state_by_axis) != set(_AXIS_ORDER):
        raise ValueError("plan-base requires exactly one state for each of five axes")
    positions: dict[AxisName, float] = {}
    for axis in _AXIS_ORDER:
        state = state_by_axis[axis]
        if not state.connected:
            raise ValueError(f"axis {axis.value} is not connected")
        if state.faulted:
            raise ValueError(
                f"axis {axis.value} is faulted: {state.fault_code!r} "
                f"{state.fault_message or ''}".rstrip()
            )
        if state.busy is not False:
            raise ValueError(f"axis {axis.value} busy is not confirmed false")
        if not state.position_valid or state.current_position is None:
            raise ValueError(f"axis {axis.value} current position is not valid")
        if axis in _LINEAR_AXES and state.homed is not True:
            raise ValueError(f"axis {axis.value} is not reference-homed")
        positions[axis] = float(state.current_position)
    return RobotAxisState(
        positions[AxisName.SLIDE],
        positions[AxisName.Z],
        positions[AxisName.SHOULDER],
        positions[AxisName.ELBOW],
        positions[AxisName.ROTATION],
    )


def _emit_transform(
    transform: RigidTransform,
    *,
    emit: Callable[[str], None],
) -> None:
    xyz = transform.translation_mm
    rpy = transform.rpy_deg
    emit(f"  xyz_mm: [{xyz[0]:.9f}, {xyz[1]:.9f}, {xyz[2]:.9f}]")
    emit(f"  rpy_deg: [{rpy[0]:.9f}, {rpy[1]:.9f}, {rpy[2]:.9f}]")


if __name__ == "__main__":
    raise SystemExit(main())
