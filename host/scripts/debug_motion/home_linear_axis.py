#!/usr/bin/env python3
"""Public Slide/Z reference-homing debug interface.

Reference home is a real mechanical motion.
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
    MotionCommandStatus,
)
from scripts._motion_cli_common import (  # noqa: E402
    best_effort_stop_axes_once,
    create_configured_runtime,
    format_axis_state,
    format_command_result,
    positive_float,
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


def run_home_test(
    runtime: object,
    axis: AxisName,
    *,
    execute: bool,
    timeout_s: float,
    emit: Callable[[str], None] = print,
) -> bool:
    """执行只读预检或一次统一机械归零，不重复 controller 的 terminal stop。"""

    if axis not in (AxisName.SLIDE, AxisName.Z):
        raise ValueError("reference home only supports slide or z")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and greater than zero")

    command_may_be_active = False
    terminal_result_received = False
    with runtime:
        before = runtime.controller.get_state(axis)
        emit(f"before: {format_axis_state(before)}")
        emit(f"planned operation: mechanical reference home for axis={axis.value}")
        if not execute:
            emit("READ_ONLY preflight complete; no home or stop command was issued")
            return True

        blocking_reasons: list[str] = []
        if not before.connected:
            blocking_reasons.append("axis is not connected")
        if before.busy is not False:
            blocking_reasons.append("axis busy state is not confirmed false")
        # STM32 fault 2 is the expected position-invalid state before reference home.
        if before.faulted and before.fault_code != 2:
            blocking_reasons.append(
                f"blocking fault_code={before.fault_code!r}: "
                f"{before.fault_message or 'no detail'}"
            )
        if blocking_reasons:
            emit("HOME PREFLIGHT REJECTED: " + "; ".join(blocking_reasons))
            return False

        emit(f"HOME MOTION AUTHORIZED axis={axis.value} timeout={timeout_s:.3f}s")
        try:
            # home_reference() combines submit and wait. Once entered, an exception may
            # occur after submission, so the CLI conservatively owns one interrupt/error
            # stop until a terminal MotionCommandResult has been received.
            command_may_be_active = True
            result = runtime.controller.home_reference(axis, timeout_s=timeout_s)
            terminal_result_received = True
            emit(f"home result: {format_command_result(result)}")
            after = runtime.controller.get_state(axis)
            emit(f"after: {format_axis_state(after)}")
        except BaseException:
            if command_may_be_active and not terminal_result_received:
                best_effort_stop_axes_once(runtime, (axis,), emit=emit)
            raise

        succeeded = (
            result.status is MotionCommandStatus.ARRIVED
            and after.homed is True
            and after.position_valid is True
            and after.busy is False
            and not after.faulted
        )
        if succeeded:
            emit(f"HOME VERIFIED axis={axis.value}")
        else:
            emit(
                "HOME NOT VERIFIED; controller owns timeout/fault best-effort stop, "
                "so the CLI will not send a duplicate stop after this terminal result"
            )
        return succeeded


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect or explicitly reference-home one linear axis through the "
            "unified controller."
        )
    )
    parser.add_argument("--axis", choices=("slide", "z"), required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-home-motion", action="store_true")
    parser.add_argument("--timeout", type=positive_float)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
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
        "software/protocol stop is not disable or power cut"
    )
    try:
        runtime = create_configured_runtime(mode)
        return 0 if run_home_test(
            runtime,
            axis,
            execute=args.execute,
            timeout_s=timeout_s,
        ) else 1
    except KeyboardInterrupt:
        print(
            "home interrupted; the CLI attempted at most one software stop",
            file=sys.stderr,
        )
        return 130
    except Exception as exc:
        print(f"upper-motion home failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
