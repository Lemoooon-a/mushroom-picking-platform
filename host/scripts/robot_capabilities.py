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
from config.frame_transforms import (  # noqa: E402
    FrameTransformConfigError,
    load_frame_transforms_document,
)


DEFAULT_FRAME_CONFIG = HOST_ROOT / "config" / "local" / "frame_transforms.json"


def load_capabilities(frame_config: Path) -> RobotCapabilities:
    document = load_frame_transforms_document(frame_config)
    calibration = hand_eye_from_frame_document(
        document,
        source=str(frame_config),
    )
    hand_eye_available = calibration is not None and calibration.validated
    return RobotCapabilities(
        base_frame_motion=document.metadata.get("validated") is True,
        suction_control=True,
        rotary_joint_enable_control=True,
        hand_eye_calibration=hand_eye_status(calibration),
        vision_target_resolution=hand_eye_available,
        vision_target_motion=(
            hand_eye_available and document.metadata.get("validated") is True
        ),
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
    parser.add_argument("--frame-config", type=Path, default=DEFAULT_FRAME_CONFIG)
    args = parser.parse_args(argv)
    try:
        for line in format_capabilities(load_capabilities(args.frame_config)):
            print(line)
        return 0
    except (FrameTransformConfigError, OSError, TypeError, ValueError) as exc:
        print(f"capability configuration error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
