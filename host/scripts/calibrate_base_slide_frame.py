#!/usr/bin/env python3
"""只读采集当前五轴状态并预览/保存 Base–Slide-zero 标定。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from bootstrap import UpperMotionRuntime, create_upper_motion_runtime  # noqa: E402
from calibration.base_slide_calibration import (  # noqa: E402
    BaseSlideCalibrationInput,
    BaseSlideCalibrationResult,
    calibrate_base_T_slide_zero,
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
    FixedFrameTransforms,
    FrameTransformConfigError,
    load_frame_transforms_document,
    save_frame_transforms,
)
from config.hardware import (  # noqa: E402
    HardwareConfigLoadError,
    load_local_hardware_config,
)
from config.motion_runtime import (  # noqa: E402
    MotionRuntimeConfigLoadError,
    load_local_motion_config,
)
from geometry.rigid_transform import RigidTransform  # noqa: E402
from kinematics.frame_chain import (  # noqa: E402
    RobotAxisState,
    SlideZeroKinematics,
)
from kinematics.five_axis import FiveAxisKinematics  # noqa: E402
from motion.authorization import RuntimeMode  # noqa: E402


DEFAULT_LOCAL_PATH = HOST_ROOT / "config" / "local" / "frame_transforms.json"
DEFAULT_FK_PROVIDER = "kinematics.five_axis:load_local_five_axis_kinematics"


def capture_and_calibrate(
    runtime: UpperMotionRuntime | object,
    slide_zero_kinematics: SlideZeroKinematics,
    base_T_tool_reference: RigidTransform,
    *,
    expected_slide_yaw_deg: float | None = 0.0,
    max_yaw_error_deg: float = 5.0,
    max_roll_pitch_deg: float = 1.0,
    slide_zero_tolerance_mm: float = 0.5,
    z_zero_tolerance_mm: float = 0.5,
    samples: int = 20,
    sample_interval_s: float = 0.05,
    max_linear_drift_mm: float = 0.1,
    max_rotary_drift_deg: float = 0.1,
) -> tuple[RobotAxisState, BaseSlideCalibrationResult]:
    """Runtime 只执行 open/close、只读初始化和状态读取。"""

    with runtime:
        initialize_read_only_rotary_positions(runtime)
        axis_state = capture_stable_axis_state(
            runtime.controller.get_axis_states,
            samples=samples,
            sample_interval_s=sample_interval_s,
            max_linear_drift_mm=max_linear_drift_mm,
            max_rotary_drift_deg=max_rotary_drift_deg,
            require_slide_z_zero=True,
            slide_zero_tolerance_mm=slide_zero_tolerance_mm,
            z_zero_tolerance_mm=z_zero_tolerance_mm,
        )
    slide_zero_T_tool = slide_zero_kinematics.forward_kinematics(axis_state)
    if not isinstance(slide_zero_T_tool, RigidTransform):
        raise TypeError("FK provider must return RigidTransform")
    result = calibrate_base_T_slide_zero(
        BaseSlideCalibrationInput(
            base_T_tool_reference=base_T_tool_reference,
            slide_zero_T_tool_at_capture=slide_zero_T_tool,
            expected_slide_yaw_deg=expected_slide_yaw_deg,
            max_slide_yaw_error_deg=max_yaw_error_deg,
            max_roll_pitch_deg=max_roll_pitch_deg,
        )
    )
    return axis_state, result


def save_calibration_result(
    path: Path,
    *,
    axis_state: RobotAxisState,
    base_T_tool_reference: RigidTransform,
    result: BaseSlideCalibrationResult,
    force: bool,
    notes: str | None,
    git_commit: str | None,
    fk_provider: str | None = None,
    fk_geometry: dict[str, object] | None = None,
) -> None:
    """保留已有 Tool–Camera 外参；无效结果仅允许显式 force。"""

    if not result.valid and not force:
        raise ValueError("calibration result is invalid; --force is required to save")
    existing_tool = None
    metadata: dict[str, object] = {}
    if path.exists():
        document = load_frame_transforms_document(path)
        existing_tool = document.transforms.tool_T_camera
        metadata.update(document.metadata)
    timestamp = datetime.now(timezone.utc).isoformat()
    metadata.update(
        {
            "base_slide_calibrated_at": timestamp,
            "base_slide_calibration_method": "single_known_base_tcp_pose",
            "validated": False,
            "git_commit": git_commit,
            "fk_provider": fk_provider,
            "fk_geometry": fk_geometry,
            "captured_axis_positions": _axis_state_dict(axis_state),
            "reference_base_T_tool": _transform_dict(base_T_tool_reference),
            "computed_base_T_slide_zero": _transform_dict(result.base_T_slide_zero),
            "computed_slide_zero_T_base": _transform_dict(result.slide_zero_T_base),
            "calibration_checks": {
                "position_residual_mm": result.position_residual_mm,
                "yaw_residual_deg": result.yaw_residual_deg,
                "estimated_roll_deg": result.estimated_base_slide_roll_deg,
                "estimated_pitch_deg": result.estimated_base_slide_pitch_deg,
                "estimated_yaw_deg": result.estimated_base_slide_yaw_deg,
                "expected_yaw_deg": result.expected_base_slide_yaw_deg,
                "yaw_alignment_error_deg": result.slide_yaw_alignment_error_deg,
                "valid": result.valid,
                "warnings": list(result.warnings),
            },
            "operator_notes": notes,
        }
    )
    save_frame_transforms(
        path,
        FixedFrameTransforms(
            base_T_slide_zero=result.base_T_slide_zero,
            tool_T_camera=existing_tool,
        ),
        metadata=metadata,
        overwrite=force,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read current logical axis states and compute Base-to-Slide-zero; "
            "never homes, moves, stops, enables, or torque-enables an axis."
        )
    )
    for name in ("x", "y", "z", "yaw"):
        parser.add_argument(f"--tcp-{name}-mm" if name != "yaw" else "--tcp-yaw-deg", type=float, required=True)
    parser.add_argument(
        "--fk-provider",
        default=DEFAULT_FK_PROVIDER,
        help=(
            "module:attribute implementing the confirmed complete "
            "SlideZeroKinematics Protocol (default: %(default)s)"
        ),
    )
    parser.add_argument("--expected-slide-yaw-deg", type=float, default=0.0)
    parser.add_argument("--max-yaw-error-deg", type=float, default=5.0)
    parser.add_argument("--max-roll-pitch-deg", type=float, default=1.0)
    parser.add_argument("--slide-zero-tolerance-mm", type=float, default=0.5)
    parser.add_argument("--z-zero-tolerance-mm", type=float, default=0.5)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--sample-interval-s", type=float, default=0.05)
    parser.add_argument("--max-linear-drift-mm", type=float, default=0.1)
    parser.add_argument("--max-rotary-drift-deg", type=float, default=0.1)
    parser.add_argument("--write-local", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_LOCAL_PATH)
    parser.add_argument("--notes")
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    runtime_factory: Callable[[], object] | None = None,
) -> int:
    args = _build_parser().parse_args(argv)
    try:
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
        axis_state, result = capture_and_calibrate(
            runtime,
            kinematics,
            reference,
            expected_slide_yaw_deg=args.expected_slide_yaw_deg,
            max_yaw_error_deg=args.max_yaw_error_deg,
            max_roll_pitch_deg=args.max_roll_pitch_deg,
            slide_zero_tolerance_mm=args.slide_zero_tolerance_mm,
            z_zero_tolerance_mm=args.z_zero_tolerance_mm,
            samples=args.samples,
            sample_interval_s=args.sample_interval_s,
            max_linear_drift_mm=args.max_linear_drift_mm,
            max_rotary_drift_deg=args.max_rotary_drift_deg,
        )
        _print_preview(axis_state, reference, result)
        if args.write_local:
            save_calibration_result(
                args.output,
                axis_state=axis_state,
                base_T_tool_reference=reference,
                result=result,
                force=args.force,
                notes=args.notes,
                git_commit=_git_commit(),
                fk_provider=args.fk_provider,
                fk_geometry=_five_axis_geometry_dict(kinematics),
            )
            print(f"Saved local frame transforms: {args.output}")
        else:
            print("Preview only; no file was written. Use --write-local to save.")
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
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"calibration runtime error: {exc}", file=sys.stderr)
        return 2


def _print_preview(
    axis_state: RobotAxisState,
    reference: RigidTransform,
    result: BaseSlideCalibrationResult,
) -> None:
    print("Captured axis state:")
    for name, value in _axis_state_dict(axis_state).items():
        print(f"  {name}: {value:.9f}")
    _print_transform("Reference base_T_tool", reference)
    _print_transform("Computed base_T_slide_zero", result.base_T_slide_zero)
    _print_transform("Computed slide_zero_T_base", result.slide_zero_T_base)
    print("Checks:")
    print(f"  reconstruction position residual: {result.position_residual_mm:.9g} mm")
    print(f"  reconstruction yaw residual: {result.yaw_residual_deg:.9g} deg")
    print(f"  slide/base yaw alignment error: {result.slide_yaw_alignment_error_deg}")
    print(f"  roll: {result.estimated_base_slide_roll_deg:.9f} deg")
    print(f"  pitch: {result.estimated_base_slide_pitch_deg:.9f} deg")
    print(f"  valid: {result.valid}")
    print(f"  warnings: {list(result.warnings)}")


def _print_transform(label: str, transform: RigidTransform) -> None:
    xyz = transform.translation_mm
    rpy = transform.rpy_deg
    print(f"{label}:")
    print(f"  xyz_mm: [{xyz[0]:.9f}, {xyz[1]:.9f}, {xyz[2]:.9f}]")
    print(f"  rpy_deg: [{rpy[0]:.9f}, {rpy[1]:.9f}, {rpy[2]:.9f}]")


def _axis_state_dict(axis_state: RobotAxisState) -> dict[str, float]:
    return {
        "slide_mm": axis_state.slide_mm,
        "z_mm": axis_state.z_mm,
        "shoulder_deg": axis_state.shoulder_deg,
        "elbow_deg": axis_state.elbow_deg,
        "rotation_deg": axis_state.rotation_deg,
    }


def _transform_dict(transform: RigidTransform) -> dict[str, list[float]]:
    return {
        "translation_mm": [float(value) for value in transform.translation_mm],
        "rotation_rpy_deg": [float(value) for value in transform.rpy_deg],
    }


def _five_axis_geometry_dict(
    kinematics: SlideZeroKinematics,
) -> dict[str, object] | None:
    if not isinstance(kinematics, FiveAxisKinematics):
        return None
    geometry = kinematics.geometry
    return {
        "link_lengths_mm": [
            geometry.link1_length_mm,
            geometry.link2_length_mm,
        ],
        "slide_direction_xyz": list(geometry.slide_direction_xyz),
        "z_direction_xyz": list(geometry.z_direction_xyz),
        "slide_zero_T_planar_origin_at_zero": _transform_dict(
            geometry.slide_zero_T_planar_origin_at_zero
        ),
        "rotation_output_T_tool": _transform_dict(
            geometry.rotation_output_T_tool
        ),
    }


def _git_commit() -> str | None:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=HOST_ROOT.parent,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


if __name__ == "__main__":
    raise SystemExit(main())
