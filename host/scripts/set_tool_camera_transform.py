#!/usr/bin/env python3
"""预览或显式录入 ``tool_T_camera`` 固定六自由度外参。"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path
import sys

import numpy as np


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from config.frame_transforms import (  # noqa: E402
    FixedFrameTransforms,
    FrameTransformConfigError,
    load_frame_transforms_document,
    save_frame_transforms,
)
from geometry.rigid_transform import RigidTransform  # noqa: E402


DEFAULT_LOCAL_PATH = HOST_ROOT / "config" / "local" / "frame_transforms.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Set tool_T_camera (Camera coordinates to Tool coordinates). "
            "Preview only unless --write-local is supplied."
        )
    )
    parser.add_argument("--x-mm", type=float, required=True)
    parser.add_argument("--y-mm", type=float, required=True)
    parser.add_argument("--z-mm", type=float, required=True)
    parser.add_argument("--roll-deg", type=float, required=True)
    parser.add_argument("--pitch-deg", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_LOCAL_PATH)
    parser.add_argument("--write-local", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--notes")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        document = load_frame_transforms_document(args.config)
        tool_T_camera = RigidTransform.from_xyz_rpy_deg(
            x_mm=args.x_mm,
            y_mm=args.y_mm,
            z_mm=args.z_mm,
            roll_deg=args.roll_deg,
            pitch_deg=args.pitch_deg,
            yaw_deg=args.yaw_deg,
        )
        round_trip = tool_T_camera @ tool_T_camera.inverse()
        round_trip_error = float(np.max(np.abs(round_trip.matrix - np.eye(4))))
        print("tool_T_camera converts Camera coordinates into Tool coordinates")
        print(f"translation_mm: {tool_T_camera.translation_mm.tolist()}")
        print(f"rotation_rpy_deg: {tool_T_camera.rpy_deg.tolist()}")
        print(f"inverse round-trip max matrix error: {round_trip_error:.3g}")
        if args.write_local:
            if not args.force:
                raise FileExistsError(
                    "updating an existing local config requires --force"
                )
            metadata = dict(document.metadata)
            metadata.update(
                {
                    "tool_camera_set_at": datetime.now(timezone.utc).isoformat(),
                    "tool_camera_method": "manual_fixed_extrinsic_entry",
                    "tool_camera_source": "manual_entry",
                    "tool_camera_validated": False,
                    "tool_camera_notes": args.notes,
                }
            )
            save_frame_transforms(
                args.config,
                FixedFrameTransforms(
                    base_T_slide_zero=document.transforms.base_T_slide_zero,
                    tool_T_camera=tool_T_camera,
                ),
                metadata=metadata,
                overwrite=True,
            )
            print(f"Saved tool_T_camera: {args.config}")
        else:
            print("Preview only; no file was written. Use --write-local --force to save.")
        return 0
    except (
        FileExistsError,
        FrameTransformConfigError,
        TypeError,
        ValueError,
    ) as exc:
        print(f"tool-camera configuration error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
