#!/usr/bin/env python3
"""通过统一 Runtime 对 Slide 或 Z 执行受门禁保护的单轴机械归零。"""

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
from motion.unified_protocol import (  # noqa: E402
    AxisName,
    AxisState,
    MotionCommandResult,
    MotionCommandStatus,
)


_HOME_TIMEOUTS_S = {
    AxisName.SLIDE: 15.0,
    AxisName.Z: 60.0,
}

_HOME_SAFETY_NOTES = {
    AxisName.SLIDE: (
        "Slide homes right (-Y) with StallGuard after a left (+Y) pre-clear; "
        "verify at least 13.5 mm pre-clear space and a reachable right stop."
    ),
    AxisName.Z: (
        "Z homes downward (-Z) to its switch; mechanically support the load "
        "and verify the switch is reachable within about 15.6 mm."
    ),
}


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
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


def _format_result(result: MotionCommandResult) -> str:
    error_code = result.error_code.value if result.error_code is not None else None
    return (
        f"status={result.status.value} accepted={result.accepted} "
        f"completed={result.completed} final_position={result.final_position} "
        f"error_code={error_code} message={result.message!r}"
    )


def _best_effort_stop(
    runtime: UpperMotionRuntime,
    axis: AxisName,
    emit: Callable[[str], None],
) -> None:
    try:
        result = runtime.controller.stop(axis)
    except Exception as exc:
        emit(f"best-effort software stop raised: {exc}")
        return
    emit(f"best-effort software stop result: {_format_result(result)}")


def run_home_test(
    runtime: UpperMotionRuntime,
    axis: AxisName,
    *,
    execute: bool,
    timeout_s: float,
    emit: Callable[[str], None] = print,
) -> bool:
    """运行单轴只读预检或显式回零；返回是否满足成功验收条件。"""

    if axis not in (AxisName.SLIDE, AxisName.Z):
        raise ValueError("home test axis must be slide or z")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and greater than zero")

    with runtime:
        before = runtime.controller.get_state(axis)
        emit(f"before: {_format_state(before)}")
        if not execute:
            emit(
                "READ_ONLY preflight complete; no home, enable, stop, or motion "
                "command was issued"
            )
            return True

        blocking_reasons: list[str] = []
        if not before.connected:
            blocking_reasons.append("axis is not connected")
        if before.busy is not False:
            blocking_reasons.append("axis busy state is not confirmed false")
        # STM32 QS fault 2 means the expected pre-home position-invalid state.
        # Stall, driver, and homing faults must be resolved before real homing.
        if before.faulted and before.fault_code != 2:
            blocking_reasons.append(
                f"blocking axis fault_code={before.fault_code!r}: "
                f"{before.fault_message or 'no detail'}"
            )
        if blocking_reasons:
            emit("HOME PREFLIGHT REJECTED: " + "; ".join(blocking_reasons))
            return False

        emit(f"HOME MOTION AUTHORIZED for axis={axis.value} timeout={timeout_s:.3f}s")
        try:
            result = runtime.controller.home_reference(axis, timeout_s=timeout_s)
            emit(f"home result: {_format_result(result)}")
            after = runtime.controller.get_state(axis)
            emit(f"after: {_format_state(after)}")
        except BaseException:
            _best_effort_stop(runtime, axis, emit)
            raise

        succeeded = (
            result.status is MotionCommandStatus.ARRIVED
            and after.homed is True
            and after.position_valid is True
        )
        if succeeded:
            emit(f"HOME VERIFIED for axis={axis.value}")
            return True

        emit(
            f"HOME NOT VERIFIED for axis={axis.value}; attempting best-effort "
            "software stop"
        )
        _best_effort_stop(runtime, axis, emit)
        return False


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read one linear-axis state through UpperMotionRuntime, or explicitly "
            "authorize one real mechanical homing operation."
        )
    )
    parser.add_argument("--axis", choices=("slide", "z"), required=True)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="select MOTION mode; also requires --confirm-home-motion",
    )
    parser.add_argument(
        "--confirm-home-motion",
        action="store_true",
        help="confirm that this invocation may enable and mechanically move one axis",
    )
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        help="positive home timeout in seconds; defaults to 15 for slide or 60 for z",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.execute != args.confirm_home_motion:
        parser.error(
            "real homing requires both --execute and --confirm-home-motion; "
            "omit both for READ_ONLY preflight"
        )

    axis = AxisName(args.axis)
    timeout_s = args.timeout if args.timeout is not None else _HOME_TIMEOUTS_S[axis]
    mode = RuntimeMode.MOTION if args.execute else RuntimeMode.READ_ONLY
    print(_HOME_SAFETY_NOTES[axis])
    print(
        f"runtime mode={mode.value} axis={axis.value} timeout={timeout_s:.3f}s; "
        "context close is not an emergency stop"
    )
    try:
        runtime = create_upper_motion_runtime(
            load_local_hardware_config(),
            load_local_motion_config(),
            mode=mode,
        )
        return 0 if run_home_test(
            runtime,
            axis,
            execute=args.execute,
            timeout_s=timeout_s,
        ) else 1
    except (HardwareConfigLoadError, MotionRuntimeConfigLoadError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("home test interrupted; a software stop was attempted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"upper-motion home test failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
