#!/usr/bin/env python3
"""统一 Runtime 五轴协调点到点运动的受控实机测试入口。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import math
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from bootstrap import UpperMotionRuntime, create_upper_motion_runtime  # noqa: E402
from config.hardware import (  # noqa: E402
    HardwareConfigLoadError,
    load_local_hardware_config,
)
from config.motion_runtime import (  # noqa: E402
    MotionRuntimeConfigLoadError,
    load_local_motion_config,
)
from motion.authorization import RuntimeMode  # noqa: E402
from motion.unified_controller import (  # noqa: E402
    MultiAxisSubmissionError,
    UnifiedMotionError,
)
from motion.unified_protocol import (  # noqa: E402
    AxisKind,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandStatus,
    MultiAxisCommandResult,
    MultiAxisTarget,
)


_AXES = tuple(AxisName)
_LINEAR_AXES = frozenset((AxisName.SLIDE, AxisName.Z))
_CAN_AXES = frozenset((AxisName.SHOULDER, AxisName.ELBOW))
_STOPPABLE_AXES = (
    AxisName.SLIDE,
    AxisName.Z,
    AxisName.SHOULDER,
    AxisName.ELBOW,
)


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def _positive_float(value: str) -> float:
    parsed = _finite_float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def _format_state(state: AxisState) -> str:
    position = (
        "unknown"
        if state.current_position is None
        else f"{state.current_position:.6f} {state.position_unit}"
    )
    return (
        f"axis={state.axis.value} connected={state.connected} "
        f"enabled={state.enabled} busy={state.busy} homed={state.homed} "
        f"position_valid={state.position_valid} position={position} "
        f"faulted={state.faulted} fault_code={state.fault_code} "
        f"fault_message={state.fault_message!r}"
    )


def _format_group_result(result: MultiAxisCommandResult) -> tuple[str, ...]:
    lines = [
        f"group result: status={result.status.value} accepted={result.accepted} "
        f"completed={result.completed} message={result.message!r}"
    ]
    lines.extend(
        f"  axis={item.axis.value} status={item.status.value} "
        f"final_position={item.final_position} error_code="
        f"{None if item.error_code is None else item.error_code.value} "
        f"message={item.message!r}"
        for item in result.results
    )
    return tuple(lines)


def _validate_target_axes(target: MultiAxisTarget) -> None:
    axes = tuple(item.axis for item in target.targets)
    if axes != _AXES:
        raise ValueError(
            "five-axis test target must use the safety order "
            "slide, z, shoulder, elbow, rotation"
        )


def _preflight_reasons(
    runtime: UpperMotionRuntime,
    target: MultiAxisTarget,
    states: tuple[AxisState, ...],
    *,
    max_linear_delta_mm: float,
    max_rotary_delta_deg: float,
) -> tuple[str, ...]:
    reasons: list[str] = []
    state_by_axis = {state.axis: state for state in states}
    profiles = runtime.motion_config.profiles()
    if len(state_by_axis) != len(_AXES) or set(state_by_axis) != set(_AXES):
        return ("unified state query did not return exactly all five axes",)

    try:
        runtime.controller.validate_positions(target)
    except UnifiedMotionError as exc:
        reasons.append(f"unified target validation failed: {exc}")

    for item in target.targets:
        state = state_by_axis[item.axis]
        descriptor = runtime.controller.describe_axis(item.axis)
        profile = profiles[item.axis]
        if not state.connected:
            reasons.append(f"{item.axis.value}: backend is not connected")
        if state.busy is not False:
            reasons.append(f"{item.axis.value}: busy is not confirmed false")
        if state.faulted:
            reasons.append(
                f"{item.axis.value}: fault_code={state.fault_code!r} "
                f"{state.fault_message or 'without diagnostic detail'}"
            )
        if not state.position_valid or state.current_position is None:
            reasons.append(f"{item.axis.value}: current position is not valid")
            continue
        if item.axis in _LINEAR_AXES and state.homed is not True:
            reasons.append(f"{item.axis.value}: reference home is not confirmed")
        if item.axis in _CAN_AXES and state.enabled is not True:
            reasons.append(f"{item.axis.value}: motor enabled state is not confirmed")
        if descriptor.capabilities.configurable_velocity:
            if profile.default_velocity is None:
                reasons.append(
                    f"{item.axis.value}: local motion profile has no default velocity"
                )
        if descriptor.capabilities.configurable_acceleration:
            if profile.default_acceleration is None:
                reasons.append(
                    f"{item.axis.value}: local motion profile has no default acceleration"
                )
        if not descriptor.minimum_position <= item.position <= descriptor.maximum_position:
            reasons.append(
                f"{item.axis.value}: target {item.position} is outside "
                f"[{descriptor.minimum_position}, {descriptor.maximum_position}] "
                f"{descriptor.position_unit}"
            )
        maximum_delta = (
            max_linear_delta_mm
            if descriptor.kind is AxisKind.LINEAR
            else max_rotary_delta_deg
        )
        delta = abs(item.position - state.current_position)
        if delta > maximum_delta:
            reasons.append(
                f"{item.axis.value}: requested delta {delta:.6f} "
                f"{descriptor.position_unit} exceeds invocation limit "
                f"{maximum_delta:.6f} {descriptor.position_unit}"
            )
    return tuple(reasons)


def _format_motion_plan(
    runtime: UpperMotionRuntime,
    item: AxisTarget,
) -> str:
    descriptor = runtime.controller.describe_axis(item.axis)
    profile = runtime.motion_config.profiles()[item.axis]
    if item.axis is AxisName.ROTATION:
        return (
            f"speed_raw={runtime.rotation_axis.config.max_speed_raw} "
            "(rotation config maximum; physical deg/s unverified) "
            "acceleration=unsupported"
        )

    velocity = item.velocity if item.velocity is not None else profile.default_velocity
    velocity_source = "explicit" if item.velocity is not None else "default"
    assert velocity is not None
    if item.axis in _LINEAR_AXES:
        acceleration = (
            item.acceleration
            if item.acceleration is not None
            else profile.default_acceleration
        )
        acceleration_source = (
            "explicit" if item.acceleration is not None else "default"
        )
        assert acceleration is not None
        maximum_velocity, maximum_acceleration = (
            runtime.motion_config.linear_motion_limits()[item.axis]
        )
        return (
            f"velocity={velocity:.6f} {descriptor.velocity_unit} "
            f"source={velocity_source} limit={maximum_velocity:.6f} "
            f"{descriptor.velocity_unit} acceleration={acceleration:.6f} "
            f"{descriptor.acceleration_unit} source={acceleration_source} "
            f"limit={maximum_acceleration:.6f} {descriptor.acceleration_unit}"
        )

    backend = (
        runtime.shoulder_joint
        if item.axis is AxisName.SHOULDER
        else runtime.elbow_joint
    )
    maximum_velocity = math.degrees(backend.config.max_velocity_rad_s)
    return (
        f"velocity={velocity:.6f} {descriptor.velocity_unit} "
        f"source={velocity_source} backend_limit={maximum_velocity:.6f} "
        f"{descriptor.velocity_unit} acceleration=unsupported"
    )


def _best_effort_stop_stoppable_axes(
    runtime: UpperMotionRuntime,
    emit: Callable[[str], None],
) -> None:
    for axis in _STOPPABLE_AXES:
        try:
            result = runtime.controller.stop(axis)
            emit(
                f"best-effort stop axis={axis.value}: status={result.status.value} "
                f"message={result.message!r}"
            )
        except Exception as exc:
            emit(f"best-effort stop axis={axis.value} failed: {exc}")
    emit(
        "Rotation was not stopped: no verified independent Rotation stop exists; "
        "use the physical emergency stop if motion is unsafe"
    )


def run_five_axis_test(
    runtime: UpperMotionRuntime,
    target: MultiAxisTarget,
    *,
    execute: bool,
    timeout_s: float,
    max_linear_delta_mm: float,
    max_rotary_delta_deg: float,
    emit: Callable[[str], None] = print,
) -> bool:
    """执行只读五轴预检，或在显式授权后提交一次五轴点到点目标。"""

    _validate_target_axes(target)
    for value, name in (
        (timeout_s, "timeout_s"),
        (max_linear_delta_mm, "max_linear_delta_mm"),
        (max_rotary_delta_deg, "max_rotary_delta_deg"),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and greater than zero")

    with runtime:
        shoulder = runtime.shoulder_joint.initialize()
        elbow = runtime.elbow_joint.initialize()
        emit(
            "CAN absolute-position initialization complete: "
            f"shoulder={math.degrees(shoulder.position_rad):.6f} deg "
            f"elbow={math.degrees(elbow.position_rad):.6f} deg"
        )

        before = runtime.controller.get_axis_states(_AXES)
        emit("before:")
        for state in before:
            emit(f"  {_format_state(state)}")

        reasons = _preflight_reasons(
            runtime,
            target,
            before,
            max_linear_delta_mm=max_linear_delta_mm,
            max_rotary_delta_deg=max_rotary_delta_deg,
        )
        if reasons:
            emit("FIVE-AXIS PREFLIGHT REJECTED:")
            for reason in reasons:
                emit(f"  - {reason}")
            return False

        state_by_axis = {state.axis: state for state in before}
        emit("planned targets:")
        for item in target.targets:
            state = state_by_axis[item.axis]
            assert state.current_position is not None
            emit(
                f"  axis={item.axis.value} current={state.current_position:.6f} "
                f"target={item.position:.6f} delta="
                f"{item.position - state.current_position:+.6f} "
                f"{state.position_unit} {_format_motion_plan(runtime, item)}"
            )

        if not execute:
            emit(
                "READ_ONLY five-axis preflight complete; no goal, torque-enable, "
                "stop, home, or motion command was issued"
            )
            return True

        rotation_state = state_by_axis[AxisName.ROTATION]
        assert rotation_state.current_position is not None
        rotation_current_rad = math.radians(rotation_state.current_position)
        runtime.rotation_axis.command_position(
            rotation_current_rad,
            runtime.rotation_axis.config.max_speed_raw,
        )
        runtime.rotation_axis.enable_torque()
        emit(
            "Rotation goal was preloaded to its current angle, then torque was "
            "explicitly enabled; torque will remain enabled after this script"
        )

        try:
            handle = runtime.controller.submit_positions(target)
        except MultiAxisSubmissionError as exc:
            for line in _format_group_result(exc.result):
                emit(line)
            emit(
                "five-axis submission failed; the controller already attempted "
                "its partial-submission stop policy"
            )
            return False
        except BaseException:
            _best_effort_stop_stoppable_axes(runtime, emit)
            raise

        try:
            result = runtime.controller.wait_group(handle, timeout_s=timeout_s)
        except BaseException:
            _best_effort_stop_stoppable_axes(runtime, emit)
            raise

        for line in _format_group_result(result):
            emit(line)
        group_arrived = (
            result.status is MotionCommandStatus.ARRIVED
            and len(result.results) == len(_AXES)
            and {item.axis for item in result.results} == set(_AXES)
            and all(
                item.status is MotionCommandStatus.ARRIVED for item in result.results
            )
        )
        if not group_arrived:
            emit(
                "FIVE-AXIS MOTION NOT VERIFIED; wait_group already applied its "
                "failure/timeout policy. Rotation may still be moving because it "
                "has no verified independent stop."
            )
            return False

        after = runtime.controller.get_axis_states(_AXES)
        emit("after:")
        for state in after:
            emit(f"  {_format_state(state)}")
        final_state_valid = all(
            state.connected
            and state.busy is False
            and state.position_valid
            and not state.faulted
            and (state.axis not in _LINEAR_AXES or state.homed is True)
            and (state.axis not in _CAN_AXES or state.enabled is True)
            for state in after
        )
        if not final_state_valid:
            emit("FIVE-AXIS ARRIVAL REPORTED, but final unified state is not healthy")
            return False
        emit("FIVE-AXIS POINT-TO-POINT MOTION VERIFIED")
        return True


def _build_target(args: argparse.Namespace) -> MultiAxisTarget:
    return MultiAxisTarget(
        (
            AxisTarget(
                AxisName.SLIDE,
                args.slide_mm,
                args.slide_speed_mm_s,
                args.slide_accel_mm_s2,
            ),
            AxisTarget(
                AxisName.Z,
                args.z_mm,
                args.z_speed_mm_s,
                args.z_accel_mm_s2,
            ),
            AxisTarget(
                AxisName.SHOULDER,
                args.shoulder_deg,
                args.shoulder_speed_deg_s,
            ),
            AxisTarget(
                AxisName.ELBOW,
                args.elbow_deg,
                args.elbow_speed_deg_s,
            ),
            AxisTarget(AxisName.ROTATION, args.rotation_deg),
        )
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read and validate a five-axis target, or explicitly execute one "
            "coordinated point-to-point submission. This is not interpolated or "
            "strictly synchronized motion."
        )
    )
    parser.add_argument("--slide-mm", type=_finite_float, required=True)
    parser.add_argument("--z-mm", type=_finite_float, required=True)
    parser.add_argument("--shoulder-deg", type=_finite_float, required=True)
    parser.add_argument("--elbow-deg", type=_finite_float, required=True)
    parser.add_argument("--rotation-deg", type=_finite_float, required=True)
    parser.add_argument("--slide-speed-mm-s", type=_positive_float)
    parser.add_argument("--slide-accel-mm-s2", type=_positive_float)
    parser.add_argument("--z-speed-mm-s", type=_positive_float)
    parser.add_argument("--z-accel-mm-s2", type=_positive_float)
    parser.add_argument("--shoulder-speed-deg-s", type=_positive_float)
    parser.add_argument("--elbow-speed-deg-s", type=_positive_float)
    parser.add_argument("--timeout", type=_positive_float, required=True)
    parser.add_argument(
        "--max-linear-delta-mm",
        type=_positive_float,
        required=True,
        help="maximum permitted absolute delta for each linear axis",
    )
    parser.add_argument(
        "--max-rotary-delta-deg",
        type=_positive_float,
        required=True,
        help="maximum permitted absolute delta for each rotary axis",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-five-axis-motion", action="store_true")
    parser.add_argument("--confirm-emergency-stop-ready", action="store_true")
    parser.add_argument("--accept-nonstrict-synchronization", action="store_true")
    parser.add_argument("--accept-unverified-rotation-stop", action="store_true")
    parser.add_argument("--enable-rotation-torque", action="store_true")
    return parser


def _validate_authorization_flags(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> None:
    confirmations = {
        "--confirm-five-axis-motion": args.confirm_five_axis_motion,
        "--confirm-emergency-stop-ready": args.confirm_emergency_stop_ready,
        "--accept-nonstrict-synchronization": args.accept_nonstrict_synchronization,
        "--accept-unverified-rotation-stop": args.accept_unverified_rotation_stop,
        "--enable-rotation-torque": args.enable_rotation_torque,
    }
    if args.execute:
        missing = tuple(name for name, enabled in confirmations.items() if not enabled)
        if missing:
            parser.error("--execute also requires " + ", ".join(missing))
    elif any(confirmations.values()):
        parser.error("motion confirmation flags require --execute")


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_authorization_flags(parser, args)
    mode = RuntimeMode.MOTION if args.execute else RuntimeMode.READ_ONLY
    print(
        "This command coordinates five point-to-point submissions back-to-back; "
        "it does not guarantee a collision-free path, interpolation, strict "
        "synchronization, or simultaneous arrival."
    )
    print(
        f"runtime mode={mode.value}; context close is not a stop; Rotation has no "
        "verified independent software stop"
    )
    try:
        runtime = create_upper_motion_runtime(
            load_local_hardware_config(),
            load_local_motion_config(),
            mode=mode,
            allow_unverified_rotation_motion=args.accept_unverified_rotation_stop,
        )
        succeeded = run_five_axis_test(
            runtime,
            _build_target(args),
            execute=args.execute,
            timeout_s=args.timeout,
            max_linear_delta_mm=args.max_linear_delta_mm,
            max_rotary_delta_deg=args.max_rotary_delta_deg,
        )
    except (HardwareConfigLoadError, MotionRuntimeConfigLoadError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("five-axis test interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"five-axis test failed: {exc}", file=sys.stderr)
        return 1
    return 0 if succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
