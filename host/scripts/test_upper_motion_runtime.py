#!/usr/bin/env python3
"""三类上层运动硬件的默认只读联合 smoke test。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
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
from motion.unified_protocol import AxisName, AxisState  # noqa: E402


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
        f"faulted={state.faulted} fault_code={state.fault_code}"
    )


def run_read_only_smoke(
    runtime: UpperMotionRuntime,
    *,
    emit: Callable[[str], None] = print,
) -> None:
    """只执行版本、绝对位置初始化和状态读取，不执行控制写入。"""

    with runtime:
        version = runtime.stm32_client.version()
        emit(
            "stm32: "
            f"protocol={version.protocol_version} "
            f"firmware={version.firmware_version}"
        )

        shoulder_initial = runtime.shoulder_joint.initialize()
        emit(
            "shoulder initialize (read-only): "
            f"position_rad={shoulder_initial.position_rad:.9f}"
        )
        elbow_initial = runtime.elbow_joint.initialize()
        emit(
            "elbow initialize (read-only): "
            f"position_rad={elbow_initial.position_rad:.9f}"
        )

        states = runtime.controller.get_axis_states(
            (
                AxisName.SLIDE,
                AxisName.Z,
                AxisName.SHOULDER,
                AxisName.ELBOW,
                AxisName.ROTATION,
            )
        )
        emit("unified axis state summary:")
        for state in states:
            emit(f"  {_format_state(state)}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Open all upper-motion communication resources and perform only "
            "read-only version/position/state checks."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="select MOTION authorization; this script still sends no motion",
    )
    parser.add_argument(
        "--allow-rotation-motion",
        action="store_true",
        help=(
            "accept the unverified Rotation-stop risk; requires --execute and "
            "still sends no motion"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.allow_rotation_motion and not args.execute:
        parser.error("--allow-rotation-motion requires --execute")

    mode = RuntimeMode.MOTION if args.execute else RuntimeMode.READ_ONLY
    try:
        runtime = create_upper_motion_runtime(
            load_local_hardware_config(),
            load_local_motion_config(),
            mode=mode,
            allow_unverified_rotation_motion=args.allow_rotation_motion,
        )
        print(
            f"runtime mode={mode.value}; this smoke test issues no motion, "
            "home, enable, torque-enable, or fault-clear command"
        )
        run_read_only_smoke(runtime)
    except (HardwareConfigLoadError, MotionRuntimeConfigLoadError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"upper-motion read-only smoke test failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
