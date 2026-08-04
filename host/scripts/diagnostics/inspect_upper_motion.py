#!/usr/bin/env python3
"""Read-only diagnostic.

Opens communication resources but does not issue motion or actuator-write commands.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[2]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from motion.authorization import RuntimeMode  # noqa: E402
from motion.unified_protocol import AxisName  # noqa: E402
from scripts._motion_cli_common import (  # noqa: E402
    create_configured_runtime,
    format_axis_descriptor,
    format_axis_state,
    initialize_read_only_rotary_positions,
)


def run_read_only_inspection(
    runtime: object,
    *,
    emit: Callable[[str], None] = print,
) -> None:
    """打开 runtime 并执行版本、只读位置初始化、descriptor 和状态查询。"""

    axes = tuple(AxisName)
    with runtime:
        version = runtime.stm32_client.version()
        emit(
            "stm32: "
            f"protocol={version.protocol_version} firmware={version.firmware_version}"
        )
        initialize_read_only_rotary_positions(runtime, axes)
        descriptors = runtime.controller.list_axes()
        states = runtime.controller.get_axis_states(axes)
        emit("unified axis descriptors:")
        for descriptor in descriptors:
            emit(f"  {format_axis_descriptor(descriptor)}")
        emit("unified axis states:")
        for state in states:
            emit(f"  {format_axis_state(state)}")


# 兼容旧 import 名称；实现仍只有上面的只读函数一份。
run_read_only_smoke = run_read_only_inspection


def build_parser() -> argparse.ArgumentParser:
    return argparse.ArgumentParser(
        description=(
            "Open upper-motion communication resources and print only read-only "
            "version, descriptor, position, and state information."
        )
    )


def main(argv: Sequence[str] | None = None) -> int:
    build_parser().parse_args(argv)
    try:
        runtime = create_configured_runtime(RuntimeMode.READ_ONLY)
        print(
            "READ_ONLY diagnostic: communication resources will be opened; no move, "
            "home, stop, enable, torque write, fault clear, suction, or config write "
            "will be issued."
        )
        run_read_only_inspection(runtime)
    except Exception as exc:
        print(f"upper-motion read-only inspection failed: {exc}", file=sys.stderr)
        return 2
    return 0


def legacy_main(argv: Sequence[str] | None = None) -> int:
    """解析旧 smoke 参数；两项旧授权参数不再改变只读行为。"""

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--allow-rotation-motion", action="store_true")
    args, remaining = parser.parse_known_args(argv)
    if args.allow_rotation_motion and not args.execute:
        parser.error("--allow-rotation-motion requires --execute")
    if args.execute:
        print(
            "DEPRECATED OPTION: --execute/--allow-rotation-motion are ignored because "
            "inspection is always READ_ONLY.",
            file=sys.stderr,
        )
    return main(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
