#!/usr/bin/env python3
"""用第二个已知 Base TCP 位姿只读验证固定变换。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from datetime import datetime, timezone
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from bootstrap import create_upper_motion_runtime  # noqa: E402
from calibration.base_slide_calibration import (  # noqa: E402
    BaseSlideVerificationResult,
    verify_base_T_slide_zero,
)
from calibration.fk_provider import (  # noqa: E402
    FKProviderLoadError,
    load_slide_zero_kinematics,
)
from calibration.state_capture import (  # noqa: E402
    AxisCaptureError,
    capture_stable_axis_state,
    initialize_read_only_rotary_positions,
)
from config.frame_transforms import (  # noqa: E402
    FrameTransformConfigError,
    load_frame_transforms_document,
    save_frame_transforms,
)
from config.hardware import HardwareConfigLoadError, load_local_hardware_config  # noqa: E402
from config.motion_runtime import (  # noqa: E402
    MotionRuntimeConfigLoadError,
    load_local_motion_config,
)
from geometry.rigid_transform import RigidTransform  # noqa: E402
from kinematics.frame_chain import RobotAxisState, SlideZeroKinematics  # noqa: E402
from motion.authorization import RuntimeMode  # noqa: E402


DEFAULT_LOCAL_PATH = HOST_ROOT / "config" / "local" / "frame_transforms.json"
DEFAULT_FK_PROVIDER = "kinematics.five_axis:load_local_five_axis_kinematics"


def capture_and_verify(
    runtime: object,
    slide_zero_kinematics: SlideZeroKinematics,
    *,
    base_T_slide_zero: RigidTransform,
    base_T_tool_reference: RigidTransform,
    max_position_error_mm: float,
    max_yaw_error_deg: float,
    samples: int = 20,
    sample_interval_s: float = 0.05,
    max_linear_drift_mm: float = 0.1,
    max_rotary_drift_deg: float = 0.1,
) -> tuple[RobotAxisState, BaseSlideVerificationResult]:
    with runtime:
        initialize_read_only_rotary_positions(runtime)
        axis_state = capture_stable_axis_state(
            runtime.kinematics_motion.get_axis_states,
            samples=samples,
            sample_interval_s=sample_interval_s,
            max_linear_drift_mm=max_linear_drift_mm,
            max_rotary_drift_deg=max_rotary_drift_deg,
            require_slide_z_zero=False,
        )
    slide_zero_T_tool = slide_zero_kinematics.forward_kinematics(axis_state)
    if not isinstance(slide_zero_T_tool, RigidTransform):
        raise TypeError("FK provider must return RigidTransform")
    return axis_state, verify_base_T_slide_zero(
        base_T_slide_zero=base_T_slide_zero,
        slide_zero_T_tool_at_capture=slide_zero_T_tool,
        base_T_tool_reference=base_T_tool_reference,
        max_position_error_mm=max_position_error_mm,
        max_yaw_error_deg=max_yaw_error_deg,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read a second stationary pose and validate Base-to-Slide-zero; "
            "never issues a motion command."
        )
    )
    for name in ("x", "y", "z"):
        parser.add_argument(f"--tcp-{name}-mm", type=float, required=True)
    parser.add_argument("--tcp-yaw-deg", type=float, required=True)
    parser.add_argument("--fk-provider", default=DEFAULT_FK_PROVIDER)
    parser.add_argument("--config", type=Path, default=DEFAULT_LOCAL_PATH)
    parser.add_argument("--max-position-error-mm", type=float, default=2.0)
    parser.add_argument("--max-yaw-error-deg", type=float, default=2.0)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--sample-interval-s", type=float, default=0.05)
    parser.add_argument("--max-linear-drift-mm", type=float, default=0.1)
    parser.add_argument("--max-rotary-drift-deg", type=float, default=0.1)
    parser.add_argument("--write-validation", action="store_true")
    parser.add_argument("--force", action="store_true")
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    runtime_factory: Callable[[], object] | None = None,
) -> int:
    args = _build_parser().parse_args(argv)
    try:
        document = load_frame_transforms_document(args.config)
        kinematics = load_slide_zero_kinematics(args.fk_provider)
        runtime = (
            runtime_factory()
            if runtime_factory is not None
            else create_upper_motion_runtime(
                load_local_hardware_config(),
                load_local_motion_config(),
                mode=RuntimeMode.READ_ONLY,
            )
        )
        reference = RigidTransform.from_xyz_yaw_deg(
            x_mm=args.tcp_x_mm,
            y_mm=args.tcp_y_mm,
            z_mm=args.tcp_z_mm,
            yaw_deg=args.tcp_yaw_deg,
        )
        axis_state, result = capture_and_verify(
            runtime,
            kinematics,
            base_T_slide_zero=document.transforms.base_T_slide_zero,
            base_T_tool_reference=reference,
            max_position_error_mm=args.max_position_error_mm,
            max_yaw_error_deg=args.max_yaw_error_deg,
            samples=args.samples,
            sample_interval_s=args.sample_interval_s,
            max_linear_drift_mm=args.max_linear_drift_mm,
            max_rotary_drift_deg=args.max_rotary_drift_deg,
        )
        _print_result(axis_state, result, bool(document.metadata.get("validated", False)))
        if args.write_validation:
            if not result.valid:
                raise ValueError("failed verification cannot be saved as validated")
            if not args.force:
                raise FileExistsError(
                    "updating an existing local config requires --force"
                )
            metadata = dict(document.metadata)
            metadata.update(
                {
                    "validated": True,
                    "base_slide_validated_at": datetime.now(timezone.utc).isoformat(),
                    "validation_fk_provider": args.fk_provider,
                    "validation_axis_positions": {
                        name: getattr(axis_state, name)
                        for name in axis_state.__dataclass_fields__
                    },
                    "validation_reference_base_T_tool": {
                        "translation_mm": [float(v) for v in reference.translation_mm],
                        "rotation_rpy_deg": [float(v) for v in reference.rpy_deg],
                    },
                    "validation_errors": {
                        "position_xyz_mm": list(result.position_error_xyz_mm),
                        "position_mm": result.position_error_mm,
                        "yaw_deg": result.yaw_error_deg,
                    },
                }
            )
            save_frame_transforms(
                args.config,
                document.transforms,
                metadata=metadata,
                overwrite=True,
            )
            print(f"Saved validation metadata: {args.config}")
        else:
            print("Read-only verification; metadata was not changed.")
        return 0 if result.valid else 1
    except (
        AxisCaptureError,
        FKProviderLoadError,
        FrameTransformConfigError,
        HardwareConfigLoadError,
        MotionRuntimeConfigLoadError,
        FileExistsError,
        TypeError,
        ValueError,
    ) as exc:
        print(f"verification configuration/state error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"verification runtime error: {exc}", file=sys.stderr)
        return 2


def _print_result(
    axis_state: RobotAxisState,
    result: BaseSlideVerificationResult,
    was_validated: bool,
) -> None:
    print(f"Previously validated metadata: {was_validated}")
    print(f"Captured axis state: {axis_state}")
    x, y, z = result.position_error_xyz_mm
    print(f"position error xyz: [{x:+.9f}, {y:+.9f}, {z:+.9f}] mm")
    print(f"3D position error: {result.position_error_mm:.9f} mm")
    print(f"yaw error: {result.yaw_error_deg:+.9f} deg")
    print(f"passed: {result.valid}")
    print(f"warnings: {list(result.warnings)}")


if __name__ == "__main__":
    raise SystemExit(main())
