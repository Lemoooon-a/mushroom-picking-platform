#!/usr/bin/env python3
"""只读输出整机应用层 capability，不打开任何硬件。"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from application.controller import RobotCapabilities  # noqa: E402
from calibration.hand_eye import (  # noqa: E402
    hand_eye_from_frame_document,
    hand_eye_status,
)
from config.robot_runtime import (  # noqa: E402
    DEFAULT_ROBOT_RUNTIME_PATH,
    RobotRuntimeConfigError,
    load_robot_runtime_config,
)
from kinematics.five_axis import (  # noqa: E402
    FiveAxisGeometryError,
    load_robot_five_axis_kinematics,
)


def load_capabilities(config_path: Path) -> RobotCapabilities:
    runtime_config = load_robot_runtime_config(config_path)
    try:
        load_robot_five_axis_kinematics()
    except FiveAxisGeometryError:
        base_frame_motion = False
    else:
        base_frame_motion = True
    calibration = hand_eye_from_frame_document(
        runtime_config.frame_transforms,
        source=f"{config_path}#frame_transforms",
    )
    hand_eye_available = calibration is not None and calibration.validated
    return RobotCapabilities(
        base_frame_motion=base_frame_motion,
        suction_control=True,
        rotary_joint_enable_control=True,
        hand_eye_calibration=hand_eye_status(calibration),
        vision_target_resolution=hand_eye_available,
        vision_target_motion=hand_eye_available and base_frame_motion,
    )


def format_capabilities(capabilities: RobotCapabilities) -> tuple[str, ...]:
    available = lambda value: "available" if value else "unavailable"
    return (
        f"Base-frame motion: {available(capabilities.base_frame_motion)}",
        f"Suction control: {available(capabilities.suction_control)}",
        "Rotary joint holding: "
        f"{available(capabilities.rotary_joint_enable_control)}",
        f"Hand-eye calibration: {capabilities.hand_eye_calibration.value}",
        "Vision target resolution: "
        f"{available(capabilities.vision_target_resolution)}",
        f"Vision target motion: {available(capabilities.vision_target_motion)}",
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Report Base-frame and vision-target application capabilities"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_ROBOT_RUNTIME_PATH)
    args = parser.parse_args(argv)
    try:
        for line in format_capabilities(load_capabilities(args.config)):
            print(line)
        return 0
    except (RobotRuntimeConfigError, OSError, TypeError, ValueError) as exc:
        print(f"capability configuration error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
