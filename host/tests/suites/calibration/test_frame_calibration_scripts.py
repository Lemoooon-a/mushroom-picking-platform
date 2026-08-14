from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest

import numpy as np

from calibration.base_slide_calibration import (
    BaseSlideCalibrationInput,
    calibrate_base_T_slide_zero,
)
from calibration.state_capture import AxisCaptureError, capture_stable_axis_state
from config.frame_transforms import (
    FixedFrameTransforms,
)
from config.robot_runtime import load_robot_runtime_config
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import AxisName, AxisState
from scripts.calibrate_base_slide_frame import (
    capture_and_calibrate,
    main as calibrate_main,
    save_calibration_result,
)
from scripts.set_tool_camera_transform import main as tool_camera_main
from scripts.verify_base_slide_frame import main as verify_main
from tests.helpers.robot_runtime_config import write_robot_runtime_fixture


def make_state(
    axis: AxisName,
    position: float,
    *,
    busy: bool | None = False,
    homed: bool | None = None,
    position_valid: bool = True,
    faulted: bool = False,
) -> AxisState:
    return AxisState(
        axis=axis,
        connected=True,
        enabled=True,
        busy=busy,
        homed=(True if axis in (AxisName.SLIDE, AxisName.Z) else homed),
        position_valid=position_valid,
        current_position=position if position_valid else None,
        position_unit=("mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg"),
        faulted=faulted,
        fault_code=(1 if faulted else None),
        fault_message=("fault" if faulted else None),
    )


def snapshot(
    *,
    slide: float = 0,
    z: float = 0,
    shoulder: float = 10,
    elbow: float = 20,
    rotation: float = 30,
    busy: bool | None = False,
) -> tuple[AxisState, ...]:
    return (
        make_state(AxisName.SLIDE, slide, busy=busy),
        make_state(AxisName.Z, z, busy=busy),
        make_state(AxisName.SHOULDER, shoulder, busy=busy),
        make_state(AxisName.ELBOW, elbow, busy=busy),
        make_state(AxisName.ROTATION, rotation, busy=busy),
    )


class SequenceReader:
    def __init__(self, snapshots: list[tuple[AxisState, ...]]) -> None:
        self.snapshots = snapshots
        self.calls = 0

    def __call__(self, axes: tuple[AxisName, ...]) -> tuple[AxisState, ...]:
        value = self.snapshots[min(self.calls, len(self.snapshots) - 1)]
        self.calls += 1
        return value


class StateCaptureTests(unittest.TestCase):
    def test_stable_samples_pass(self) -> None:
        result = capture_stable_axis_state(
            SequenceReader([snapshot()] * 3),
            samples=3,
            sample_interval_s=0,
            require_slide_z_zero=True,
        )
        np.testing.assert_allclose(
            (
                result.slide_mm,
                result.z_mm,
                result.shoulder_deg,
                result.elbow_deg,
                result.rotation_deg,
            ),
            (0, 0, 10, 20, 30),
            atol=1e-12,
        )

    def test_yaw_wrap_uses_circular_mean(self) -> None:
        result = capture_stable_axis_state(
            SequenceReader(
                [
                    snapshot(rotation=179.9),
                    snapshot(rotation=-179.9),
                    snapshot(rotation=180.0),
                ]
            ),
            samples=3,
            sample_interval_s=0,
            max_rotary_drift_deg=0.2,
        )
        self.assertAlmostEqual(abs(result.rotation_deg), 180, places=8)

    def test_unknown_busy_requires_three_samples(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "at least 3"):
            capture_stable_axis_state(
                SequenceReader([snapshot(busy=None)] * 2),
                samples=2,
                sample_interval_s=0,
            )

    def test_unknown_busy_with_stable_samples_passes(self) -> None:
        capture_stable_axis_state(
            SequenceReader([snapshot(busy=None)] * 3),
            samples=3,
            sample_interval_s=0,
        )

    def test_busy_axis_is_rejected(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "busy"):
            capture_stable_axis_state(
                SequenceReader([snapshot(busy=True)]),
                samples=2,
                sample_interval_s=0,
            )

    def test_faulted_axis_is_rejected(self) -> None:
        states = list(snapshot())
        states[4] = make_state(AxisName.ROTATION, 0, faulted=True)
        with self.assertRaisesRegex(AxisCaptureError, "faulted"):
            capture_stable_axis_state(
                SequenceReader([tuple(states)]),
                samples=2,
                sample_interval_s=0,
            )

    def test_invalid_rotation_position_is_rejected(self) -> None:
        states = list(snapshot())
        states[4] = make_state(AxisName.ROTATION, 0, position_valid=False)
        with self.assertRaisesRegex(AxisCaptureError, "position is not valid"):
            capture_stable_axis_state(
                SequenceReader([tuple(states)]),
                samples=2,
                sample_interval_s=0,
            )

    def test_unhomed_slide_is_rejected(self) -> None:
        states = list(snapshot())
        states[0] = make_state(AxisName.SLIDE, 0)
        states[0] = AxisState(**{**states[0].__dict__, "homed": False})
        with self.assertRaisesRegex(AxisCaptureError, "not homed"):
            capture_stable_axis_state(
                SequenceReader([tuple(states)]),
                samples=2,
                sample_interval_s=0,
            )

    def test_slide_outside_zero_tolerance_is_rejected(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "zero tolerance"):
            capture_stable_axis_state(
                SequenceReader([snapshot(slide=1)]),
                samples=2,
                sample_interval_s=0,
                require_slide_z_zero=True,
                slide_zero_tolerance_mm=0.5,
            )

    def test_z_outside_zero_tolerance_is_rejected(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "zero tolerance"):
            capture_stable_axis_state(
                SequenceReader([snapshot(z=-1)]),
                samples=2,
                sample_interval_s=0,
                require_slide_z_zero=True,
                z_zero_tolerance_mm=0.5,
            )

    def test_unstable_linear_axis_is_rejected(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "slide is unstable"):
            capture_stable_axis_state(
                SequenceReader([snapshot(slide=0), snapshot(slide=1)]),
                samples=2,
                sample_interval_s=0,
                max_linear_drift_mm=0.1,
            )

    def test_unstable_rotary_axis_is_rejected(self) -> None:
        with self.assertRaisesRegex(AxisCaptureError, "rotation is unstable"):
            capture_stable_axis_state(
                SequenceReader([snapshot(rotation=0), snapshot(rotation=2)]),
                samples=2,
                sample_interval_s=0,
                max_rotary_drift_deg=0.1,
            )


class FakeJoint:
    def __init__(self, events: list[str], name: str) -> None:
        self.events = events
        self.name = name

    def initialize(self) -> None:
        self.events.append(f"initialize:{self.name}")


class FakeMotion:
    def __init__(self, events: list[str], states: tuple[AxisState, ...]) -> None:
        self.events = events
        self.states = states

    def get_axis_states(self, axes: tuple[AxisName, ...]) -> tuple[AxisState, ...]:
        self.events.append("get_axis_states")
        return self.states


class FakeRuntime:
    def __init__(self, states: tuple[AxisState, ...] | None = None) -> None:
        self.events: list[str] = []
        self.shoulder_joint = FakeJoint(self.events, "shoulder")
        self.elbow_joint = FakeJoint(self.events, "elbow")
        self.controller = FakeMotion(self.events, states or snapshot())

    def __enter__(self) -> "FakeRuntime":
        self.events.append("open")
        return self

    def __exit__(self, *args: object) -> None:
        self.events.append("close")

    def home(self) -> None:
        self.events.append("home")

    def move(self) -> None:
        self.events.append("move")

    def enable(self) -> None:
        self.events.append("enable")

    def enable_torque(self) -> None:
        self.events.append("enable_torque")


class ConstantKinematics:
    def __init__(self, transform: RigidTransform | None = None) -> None:
        self.transform = transform or RigidTransform.identity()

    def forward_kinematics(self, axis_state: RobotAxisState) -> RigidTransform:
        return self.transform


class CalibrationScriptTests(unittest.TestCase):
    def test_capture_uses_only_read_actions(self) -> None:
        runtime = FakeRuntime()
        axis_state, result = capture_and_calibrate(
            runtime,
            ConstantKinematics(),
            RigidTransform.identity(),
            samples=3,
            sample_interval_s=0,
        )
        self.assertAlmostEqual(axis_state.rotation_deg, 30)
        self.assertTrue(result.valid)
        self.assertNotIn("home", runtime.events)
        self.assertNotIn("move", runtime.events)
        self.assertNotIn("enable", runtime.events)
        self.assertNotIn("enable_torque", runtime.events)
        self.assertEqual(runtime.events[0], "open")
        self.assertEqual(runtime.events[-1], "close")

    def test_default_save_preserves_tool_camera_when_forced_update(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            tool = RigidTransform.from_xyz_rpy_deg(
                x_mm=1,
                y_mm=2,
                z_mm=3,
                roll_deg=4,
                pitch_deg=5,
                yaw_deg=6,
            )
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(RigidTransform.identity(), tool),
                metadata={"keep": "yes", "validated": True},
            )
            reference = RigidTransform.from_xyz_yaw_deg(
                x_mm=10, y_mm=20, z_mm=30, yaw_deg=0
            )
            result = calibrate_base_T_slide_zero(
                BaseSlideCalibrationInput(
                    base_T_tool_reference=reference,
                    slide_zero_T_tool_at_capture=RigidTransform.identity(),
                )
            )
            save_calibration_result(
                path,
                axis_state=RobotAxisState(0, 0, 1, 2, 3),
                base_T_tool_reference=reference,
                result=result,
                force=True,
                notes="test",
                git_commit="abc",
            )
            document = load_robot_runtime_config(path).frame_transforms
            np.testing.assert_allclose(document.transforms.tool_T_camera.matrix, tool.matrix)
            self.assertEqual(document.metadata["keep"], "yes")
            self.assertFalse(document.metadata["validated"])

    def test_invalid_result_requires_force_to_save(self) -> None:
        known = RigidTransform.from_xyz_yaw_deg(
            x_mm=0, y_mm=0, z_mm=0, yaw_deg=20
        )
        result = calibrate_base_T_slide_zero(
            BaseSlideCalibrationInput(
                base_T_tool_reference=known,
                slide_zero_T_tool_at_capture=RigidTransform.identity(),
                max_slide_yaw_error_deg=1,
            )
        )
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "invalid"):
                save_calibration_result(
                    Path(directory) / "frames.json",
                    axis_state=RobotAxisState(0, 0, 0, 0, 0),
                    base_T_tool_reference=known,
                    result=result,
                    force=False,
                    notes=None,
                    git_commit=None,
                )

    def test_calibrate_main_preview_writes_nothing(self) -> None:
        with self._provider_module(), tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "robot_runtime.json"
            code = self._call_silently(
                calibrate_main,
                [
                    "--tcp-x-mm", "0", "--tcp-y-mm", "0", "--tcp-z-mm", "0",
                    "--tcp-yaw-deg", "0", "--fk-provider", "test_fk_provider:KINEMATICS",
                    "--samples", "3", "--sample-interval-s", "0", "--config", str(output),
                ],
                runtime_factory=FakeRuntime,
            )
            self.assertEqual(code, 0)
            self.assertFalse(output.exists())

    def test_calibrate_main_writes_only_with_flag(self) -> None:
        with self._provider_module(), tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "robot_runtime.json"
            write_robot_runtime_fixture(output)
            code = self._call_silently(
                calibrate_main,
                [
                    "--tcp-x-mm", "0", "--tcp-y-mm", "0", "--tcp-z-mm", "0",
                    "--tcp-yaw-deg", "0", "--fk-provider", "test_fk_provider:KINEMATICS",
                    "--samples", "3", "--sample-interval-s", "0", "--config", str(output),
                    "--write-config",
                ],
                runtime_factory=FakeRuntime,
            )
            self.assertEqual(code, 0)
            self.assertTrue(output.exists())
            self.assertFalse(
                load_robot_runtime_config(output).frame_transforms.metadata["validated"]
            )

    def test_verify_main_exit_codes_and_validation_metadata(self) -> None:
        with self._provider_module(), tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(RigidTransform.identity(), None),
                metadata={"validated": False},
            )
            args = [
                "--tcp-x-mm", "0", "--tcp-y-mm", "0", "--tcp-z-mm", "0",
                "--tcp-yaw-deg", "0", "--fk-provider", "test_fk_provider:KINEMATICS",
                "--samples", "3", "--sample-interval-s", "0", "--config", str(path),
            ]
            self.assertEqual(
                self._call_silently(verify_main, args, runtime_factory=FakeRuntime),
                0,
            )
            self.assertFalse(
                load_robot_runtime_config(path).frame_transforms.metadata["validated"]
            )
            self.assertEqual(
                self._call_silently(
                    verify_main,
                    args + ["--write-validation", "--force"],
                    runtime_factory=FakeRuntime,
                ),
                0,
            )
            self.assertTrue(
                load_robot_runtime_config(path).frame_transforms.metadata["validated"]
            )

    def test_verify_main_returns_one_for_threshold_failure(self) -> None:
        with self._provider_module(), tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(RigidTransform.identity(), None),
            )
            code = self._call_silently(
                verify_main,
                [
                    "--tcp-x-mm", "10", "--tcp-y-mm", "0", "--tcp-z-mm", "0",
                    "--tcp-yaw-deg", "0", "--fk-provider", "test_fk_provider:KINEMATICS",
                    "--samples", "3", "--sample-interval-s", "0", "--config", str(path),
                    "--max-position-error-mm", "1",
                ],
                runtime_factory=FakeRuntime,
            )
            self.assertEqual(code, 1)

    def test_tool_camera_preview_and_update_preserve_base(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_runtime.json"
            base = RigidTransform.from_xyz_yaw_deg(
                x_mm=10, y_mm=20, z_mm=30, yaw_deg=40
            )
            write_robot_runtime_fixture(
                path,
                transforms=FixedFrameTransforms(base, None),
                metadata={"keep": "yes"},
            )
            args = [
                "--x-mm", "1", "--y-mm", "2", "--z-mm", "3",
                "--roll-deg", "4", "--pitch-deg", "5", "--yaw-deg", "6",
                "--config", str(path),
            ]
            self.assertEqual(self._call_silently(tool_camera_main, args), 0)
            self.assertIsNone(
                load_robot_runtime_config(path).frame_transforms.transforms.tool_T_camera
            )
            self.assertEqual(
                self._call_silently(
                    tool_camera_main,
                    args + ["--write-config", "--force"],
                ),
                0,
            )
            document = load_robot_runtime_config(path).frame_transforms
            np.testing.assert_allclose(document.transforms.base_T_slide_zero.matrix, base.matrix)
            self.assertIsNotNone(document.transforms.tool_T_camera)
            self.assertEqual(document.metadata["keep"], "yes")
            self.assertIs(document.metadata["tool_camera_validated"], False)
            self.assertEqual(document.metadata["tool_camera_source"], "manual_entry")

    def _provider_module(self):
        test_case = self

        class ProviderContext:
            def __enter__(self) -> None:
                module = types.ModuleType("test_fk_provider")
                module.KINEMATICS = ConstantKinematics()
                sys.modules[module.__name__] = module

            def __exit__(self, *args: object) -> None:
                sys.modules.pop("test_fk_provider", None)

        return ProviderContext()

    def _call_silently(self, function, args, **kwargs):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return function(args, **kwargs)


if __name__ == "__main__":
    unittest.main()
