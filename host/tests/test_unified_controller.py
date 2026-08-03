"""五轴统一异步点到点控制器的纯 fake 测试。"""

from __future__ import annotations

import math
from types import SimpleNamespace
import unittest

from config.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from drivers.stm32_motion import AxisStatus, STM32CommandSubmission, STM32Message
from motion.unified_controller import (
    MultiAxisSubmissionError,
    UnifiedMotionController,
    UnifiedMotionError,
)
from motion.unified_protocol import (
    ArrivalConfig,
    AxisName,
    AxisTarget,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisTarget,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeSTM32:
    def __init__(self) -> None:
        self.sequence = 0
        self.submissions: list[tuple[str, int, int, int]] = []
        self.events: list[STM32Message | None] = []
        self.states: list[AxisStatus] = []
        self.stop_calls: list[str] = []
        self.home_calls: list[str] = []
        self.enable_calls = 0

    def submit_move_absolute(
        self,
        axis: str,
        position_um: int,
        velocity_um_s: int,
        acceleration_um_s2: int,
    ) -> STM32CommandSubmission:
        self.submissions.append((axis, position_um, velocity_um_s, acceleration_um_s2))
        token = STM32CommandSubmission(self.sequence, axis, "MA")
        self.sequence += 1
        return token

    def submit_home(self, axis: str) -> STM32CommandSubmission:
        self.home_calls.append(axis)
        token = STM32CommandSubmission(self.sequence, axis, "HM")
        self.sequence += 1
        return token

    def poll_command(self, _token: STM32CommandSubmission) -> STM32Message | None:
        return self.events.pop(0) if self.events else None

    def query_axis(self, axis: str) -> AxisStatus:
        if self.states:
            return self.states.pop(0)
        return axis_status(axis, position_um=0, busy=False)

    def stop(self, axis: str) -> None:
        self.stop_calls.append(axis)

    def enable(self, _axis: str) -> None:
        self.enable_calls += 1


class FakeJoint:
    def __init__(self, config: object) -> None:
        self.config = config
        self.commands: list[tuple[float, float]] = []
        self.states: list[object] = []
        self.stop_calls = 0
        self.initialize_calls = 0
        self.command_error: Exception | None = None

    def command_position(self, position_rad: float, velocity_rad_s: float) -> object:
        if self.command_error is not None:
            raise self.command_error
        self.commands.append((position_rad, velocity_rad_s))
        return object()

    def get_state(self) -> object:
        if self.states:
            return self.states.pop(0)
        return joint_state(0.0)

    def stop(self) -> None:
        self.stop_calls += 1

    def initialize(self) -> None:
        self.initialize_calls += 1


class FakeRotation:
    def __init__(self) -> None:
        self.config = END_EFFECTOR_ROTATION_CONFIG
        self.commands: list[tuple[float, int]] = []
        self.feedback: list[object] = []
        self.enable_calls = 0
        self.disable_calls = 0

    def command_position(self, position_rad: float, speed_raw: int) -> int:
        self.commands.append((position_rad, speed_raw))
        return 123

    def read_feedback(self) -> object:
        if self.feedback:
            return self.feedback.pop(0)
        return rotation_feedback(0.0)

    def enable_torque(self) -> None:
        self.enable_calls += 1

    def disable_torque(self) -> None:
        self.disable_calls += 1


def axis_status(
    axis: str,
    *,
    position_um: int,
    busy: bool,
    homed: bool = True,
    valid: bool = True,
    fault: int = 0,
) -> AxisStatus:
    code = "S" if axis in ("slide", "S") else "Z"
    return AxisStatus(code, True, True, busy, homed, valid, position_um, fault)


def joint_state(
    position_deg: float,
    *,
    moving: bool | None = False,
    fault: int = 0,
    valid: bool = True,
) -> object:
    return SimpleNamespace(
        position_rad=math.radians(position_deg),
        position_valid=valid,
        moving=moving,
        motor_state=0,
        error_state=fault,
    )


def rotation_feedback(
    position_deg: float,
    *,
    moving: bool | None = False,
    error: int = 0,
) -> object:
    return SimpleNamespace(
        position_rad=math.radians(position_deg),
        moving=moving,
        error_raw=error,
    )


def arrival_configs(stable_time_s: float = 0.1) -> dict[AxisName, ArrivalConfig]:
    return {
        axis: ArrivalConfig(
            position_tolerance=0.2 if axis in (AxisName.SLIDE, AxisName.Z) else 0.5,
            stable_time_s=stable_time_s,
            poll_interval_s=0.01,
            default_timeout_s=1.0,
        )
        for axis in AxisName
    }


class ControllerTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.stm32 = FakeSTM32()
        self.shoulder = FakeJoint(SHOULDER_JOINT_CONFIG)
        self.elbow = FakeJoint(ELBOW_JOINT_CONFIG)
        self.rotation = FakeRotation()
        self.controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            arrival_configs=arrival_configs(),
            default_motion_parameters={
                AxisName.SLIDE: (2.0, 4.0),
                AxisName.Z: (1.0, 3.0),
                AxisName.SHOULDER: (5.0, None),
                AxisName.ELBOW: (6.0, None),
            },
            clock=self.clock,
            sleep=self.clock.advance,
        )


class DescriptorAndDispatchTests(ControllerTestCase):
    def test_lists_five_axes_with_public_units(self) -> None:
        descriptors = self.controller.list_axes()
        self.assertEqual(tuple(item.name for item in descriptors), tuple(AxisName))
        self.assertEqual(descriptors[0].position_unit, "mm")
        self.assertEqual(descriptors[2].position_unit, "deg")
        self.assertFalse(descriptors[-1].capabilities.stop)

    def test_get_axis_states_preserves_requested_order(self) -> None:
        states = self.controller.get_axis_states(
            (AxisName.ELBOW, AxisName.SHOULDER)
        )
        self.assertEqual(
            tuple(state.axis for state in states),
            (AxisName.ELBOW, AxisName.SHOULDER),
        )

    def test_get_axis_states_defaults_to_all_axes(self) -> None:
        states = self.controller.get_axis_states()
        self.assertEqual(tuple(state.axis for state in states), tuple(AxisName))

    def test_slide_and_z_convert_mm_to_integer_micrometres(self) -> None:
        self.controller.submit_absolute(
            AxisTarget(AxisName.SLIDE, 12.345, 1.234, 2.345)
        )
        self.assertEqual(self.stm32.submissions[0], ("slide", 12345, 1234, 2345))

        z_controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            arrival_configs=arrival_configs(),
            default_motion_parameters={AxisName.Z: (1.0, 2.0)},
            clock=self.clock,
            sleep=self.clock.advance,
        )
        z_controller.submit_absolute(AxisTarget(AxisName.Z, 1.0))
        self.assertEqual(self.stm32.submissions[-1][1], 1000)

    def test_shoulder_elbow_and_rotation_convert_degrees_to_radians(self) -> None:
        self.controller.submit_absolute(AxisTarget(AxisName.SHOULDER, 12.345, 5.5))
        self.assertAlmostEqual(self.shoulder.commands[0][0], math.radians(12.345))
        self.assertAlmostEqual(self.shoulder.commands[0][1], math.radians(5.5))

        self.controller.submit_absolute(AxisTarget(AxisName.ELBOW, -23.456, 6.5))
        self.assertAlmostEqual(self.elbow.commands[0][0], math.radians(-23.456))

        self.controller.submit_absolute(AxisTarget(AxisName.ROTATION, 7.89))
        self.assertAlmostEqual(self.rotation.commands[0][0], math.radians(7.89))
        self.assertEqual(self.rotation.commands[0][1], self.rotation.config.max_speed_raw)

    def test_missing_backend_unknown_axis_and_soft_limit_are_explicit(self) -> None:
        controller = UnifiedMotionController(
            stm32_client=None,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            arrival_configs=arrival_configs(),
            default_motion_parameters={AxisName.SLIDE: (1.0, 1.0)},
        )
        with self.assertRaises(UnifiedMotionError) as missing:
            controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        self.assertEqual(missing.exception.error_code, MotionErrorCode.BACKEND_UNAVAILABLE)
        with self.assertRaises(UnifiedMotionError) as unknown:
            self.controller.describe_axis("bogus")  # type: ignore[arg-type]
        self.assertEqual(unknown.exception.error_code, MotionErrorCode.UNKNOWN_AXIS)
        with self.assertRaises(UnifiedMotionError) as limit:
            self.controller.submit_absolute(AxisTarget(AxisName.ROTATION, 46.0))
        self.assertEqual(limit.exception.error_code, MotionErrorCode.SOFT_LIMIT)

    def test_unsupported_parameters_are_never_silently_ignored(self) -> None:
        with self.assertRaises(UnifiedMotionError) as acceleration:
            self.controller.submit_absolute(
                AxisTarget(AxisName.SHOULDER, 1.0, 2.0, 3.0)
            )
        self.assertEqual(
            acceleration.exception.error_code,
            MotionErrorCode.UNSUPPORTED_PARAMETER,
        )
        with self.assertRaises(UnifiedMotionError):
            self.controller.submit_absolute(
                AxisTarget(AxisName.ROTATION, 1.0, velocity=2.0)
            )

    def test_constructor_and_submission_do_not_enable_home_or_torque_enable(self) -> None:
        self.assertEqual(self.stm32.submissions, [])
        self.assertEqual(self.shoulder.commands, [])
        self.assertEqual(self.elbow.commands, [])
        self.assertEqual(self.rotation.commands, [])
        self.controller.submit_absolute(AxisTarget(AxisName.SHOULDER, 1.0, 2.0))
        self.assertEqual(self.stm32.enable_calls, 0)
        self.assertEqual(self.stm32.home_calls, [])
        self.assertEqual(self.shoulder.initialize_calls, 0)
        self.assertEqual(self.rotation.enable_calls, 0)

    def test_missing_default_motion_profile_is_rejected_without_io(self) -> None:
        controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            arrival_configs=arrival_configs(),
        )
        with self.assertRaises(UnifiedMotionError) as invalid:
            controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        self.assertEqual(invalid.exception.error_code, MotionErrorCode.INVALID_REQUEST)
        self.assertEqual(self.stm32.submissions, [])

    def test_state_converts_backend_position_and_preserves_unknown_busy(self) -> None:
        self.stm32.states = [axis_status("slide", position_um=12345, busy=False)]
        self.assertEqual(
            self.controller.get_state(AxisName.SLIDE).current_position,
            12.345,
        )
        self.shoulder.states = [joint_state(12.345, moving=None)]
        shoulder_state = self.controller.get_state(AxisName.SHOULDER)
        self.assertAlmostEqual(shoulder_state.current_position or 0.0, 12.345)
        self.assertIsNone(shoulder_state.busy)
        self.rotation.feedback = [rotation_feedback(-7.89, moving=None)]
        rotation_state = self.controller.get_state(AxisName.ROTATION)
        self.assertAlmostEqual(rotation_state.current_position or 0.0, -7.89)
        self.assertIsNone(rotation_state.busy)

    def test_same_axis_rejects_second_unfinished_command(self) -> None:
        self.controller.submit_absolute(AxisTarget(AxisName.SHOULDER, 1.0, 2.0))
        with self.assertRaises(UnifiedMotionError) as busy:
            self.controller.submit_absolute(AxisTarget(AxisName.SHOULDER, 2.0, 2.0))
        self.assertEqual(busy.exception.error_code, MotionErrorCode.BUSY)


class ArrivalAndTimeoutTests(ControllerTestCase):
    def test_joint_stable_window_resets_after_leaving_tolerance(self) -> None:
        self.shoulder.states = [
            joint_state(0.0, moving=None),
            joint_state(9.8, moving=None),
            joint_state(8.0, moving=None),
            joint_state(10.1, moving=None),
            joint_state(10.1, moving=None),
        ]
        handle = self.controller.submit_absolute(
            AxisTarget(AxisName.SHOULDER, 10.0, 2.0)
        )
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        self.clock.advance(0.06)
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        self.clock.advance(0.06)
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        self.clock.advance(0.01)
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        self.clock.advance(0.11)
        result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)
        self.assertTrue(result.completed)

    def test_unknown_busy_does_not_block_position_stability_arrival(self) -> None:
        self.shoulder.states = [joint_state(5.0, moving=None)] * 2
        handle = self.controller.submit_absolute(
            AxisTarget(AxisName.SHOULDER, 5.0, 2.0)
        )
        self.controller.get_command_result(handle)
        self.clock.advance(0.11)
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.ARRIVED,
        )

    def test_fault_terminates_and_attempts_joint_software_stop(self) -> None:
        self.elbow.states = [joint_state(0.0, fault=0x40)]
        handle = self.controller.submit_absolute(
            AxisTarget(AxisName.ELBOW, 5.0, 2.0)
        )
        result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.FAULT)
        self.assertEqual(result.error_code, MotionErrorCode.DEVICE_FAULT)
        self.assertEqual(self.elbow.stop_calls, 1)

    def test_joint_timeout_uses_best_effort_software_stop(self) -> None:
        self.shoulder.states = [joint_state(0.0)] * 20
        handle = self.controller.submit_absolute(
            AxisTarget(AxisName.SHOULDER, 10.0, 2.0)
        )
        result = self.controller.wait(handle, timeout_s=0.03)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.assertEqual(self.shoulder.stop_calls, 1)
        self.assertIn("not an emergency stop", result.message)

    def test_rotation_timeout_does_not_invent_stop_or_disable_torque(self) -> None:
        self.rotation.feedback = [rotation_feedback(0.0)] * 20
        handle = self.controller.submit_absolute(AxisTarget(AxisName.ROTATION, 10.0))
        result = self.controller.wait(handle, timeout_s=0.03)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.assertIn("no independent stop", result.message)
        self.assertEqual(self.rotation.disable_calls, 0)

    def test_rotation_feedback_error_maps_to_fault(self) -> None:
        self.rotation.feedback = [rotation_feedback(0.0, error=7)]
        handle = self.controller.submit_absolute(AxisTarget(AxisName.ROTATION, 10.0))
        result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.FAULT)
        self.assertEqual(result.error_code, MotionErrorCode.DEVICE_FAULT)
        self.assertEqual(self.rotation.disable_calls, 0)

    def test_stm32_wait_timeout_stops_axis_and_returns_timeout(self) -> None:
        self.stm32.states = [axis_status("slide", position_um=0, busy=True)] * 20
        handle = self.controller.submit_absolute(AxisTarget(AxisName.SLIDE, 10.0))
        result = self.controller.wait(handle, timeout_s=0.03)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.assertEqual(self.stm32.stop_calls, ["slide"])

    def test_stm32_done_abort_and_fault_mapping(self) -> None:
        for kind, expected in (
            ("DONE", MotionCommandStatus.ARRIVED),
            ("ABORT", MotionCommandStatus.ABORTED),
            ("FAULT", MotionCommandStatus.FAULT),
        ):
            with self.subTest(kind=kind):
                stm32 = FakeSTM32()
                controller = UnifiedMotionController(
                    stm32_client=stm32,
                    shoulder_joint=self.shoulder,
                    elbow_joint=self.elbow,
                    rotation_axis=self.rotation,
                    arrival_configs=arrival_configs(),
                    default_motion_parameters={AxisName.SLIDE: (1.0, 2.0)},
                )
                handle = controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
                stm32.events.append(STM32Message("!", 0, kind, ("S",)))
                stm32.states.append(axis_status("slide", position_um=1000, busy=False))
                self.assertEqual(controller.get_command_result(handle).status, expected)

    def test_home_is_only_supported_for_linear_axes(self) -> None:
        rejected = self.controller.home_reference(AxisName.SHOULDER)
        self.assertEqual(rejected.status, MotionCommandStatus.REJECTED)
        self.stm32.events.append(STM32Message("!", 0, "DONE", ("Z",)))
        self.stm32.states.append(axis_status("z", position_um=0, busy=False))
        result = self.controller.home_reference(AxisName.Z)
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)


class MultiAxisTests(ControllerTestCase):
    def test_two_axis_submission_preserves_input_order(self) -> None:
        handle = self.controller.submit_positions(
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.ELBOW, -10.0, 3.0),
                    AxisTarget(AxisName.SHOULDER, 20.0, 4.0),
                )
            )
        )
        self.assertEqual(
            tuple(item.axis for item in handle.commands),
            (AxisName.ELBOW, AxisName.SHOULDER),
        )

    def test_get_group_result_polls_without_waiting(self) -> None:
        self.shoulder.states = [joint_state(0.0, moving=True)]
        self.elbow.states = [joint_state(0.0, moving=True)]
        handle = self.controller.submit_positions(
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                    AxisTarget(AxisName.ELBOW, -10.0, 2.0),
                )
            )
        )
        result = self.controller.get_group_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.MOVING)
        self.assertTrue(result.accepted)
        self.assertIsNone(result.completed)
        self.assertEqual(self.clock.now, 0.0)

    def test_five_axis_submission_is_back_to_back_and_complete(self) -> None:
        target = MultiAxisTarget(
            (
                AxisTarget(AxisName.SLIDE, 300.0),
                AxisTarget(AxisName.Z, 120.0),
                AxisTarget(AxisName.SHOULDER, 25.0),
                AxisTarget(AxisName.ELBOW, -60.0),
                AxisTarget(AxisName.ROTATION, 30.0),
            )
        )
        handle = self.controller.submit_positions(target)
        self.assertEqual(tuple(item.axis for item in handle.commands), tuple(AxisName))
        self.assertEqual(len(self.stm32.submissions), 2)
        self.assertEqual(len(self.shoulder.commands), 1)
        self.assertEqual(len(self.elbow.commands), 1)
        self.assertEqual(len(self.rotation.commands), 1)

    def test_partial_submission_failure_stops_already_submitted_axis(self) -> None:
        self.elbow.command_error = RuntimeError("elbow submission failed")
        with self.assertRaises(MultiAxisSubmissionError) as failure:
            self.controller.submit_positions(
                MultiAxisTarget(
                    (
                        AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                        AxisTarget(AxisName.ELBOW, -10.0, 2.0),
                        AxisTarget(AxisName.ROTATION, 1.0),
                    )
                )
            )
        result = failure.exception.result
        self.assertEqual(self.shoulder.stop_calls, 1)
        self.assertEqual(result.status, MotionCommandStatus.REJECTED)
        self.assertEqual(len(result.results), 3)
        self.assertEqual(result.results[0].status, MotionCommandStatus.ABORTED)
        self.assertEqual(result.results[1].status, MotionCommandStatus.REJECTED)
        self.assertEqual(result.results[2].status, MotionCommandStatus.REJECTED)

    def test_group_arrives_only_after_all_axes_arrive(self) -> None:
        controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            arrival_configs=arrival_configs(stable_time_s=0.0),
            default_motion_parameters={
                AxisName.SHOULDER: (2.0, None),
                AxisName.ELBOW: (2.0, None),
            },
            clock=self.clock,
            sleep=self.clock.advance,
        )
        self.shoulder.states = [joint_state(10.0)]
        self.elbow.states = [joint_state(-10.0)]
        handle = controller.submit_positions(
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.SHOULDER, 10.0),
                    AxisTarget(AxisName.ELBOW, -10.0),
                )
            )
        )
        result = controller.wait_group(handle, timeout_s=0.5)
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)
        self.assertTrue(result.completed)
        self.assertEqual(len(result.results), 2)

    def test_one_axis_fault_fails_group_and_stops_peer(self) -> None:
        self.shoulder.states = [joint_state(0.0)]
        self.elbow.states = [joint_state(0.0, fault=1)]
        handle = self.controller.submit_positions(
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                    AxisTarget(AxisName.ELBOW, -10.0, 2.0),
                )
            )
        )
        result = self.controller.wait_group(handle, timeout_s=0.5)
        self.assertEqual(result.status, MotionCommandStatus.FAULT)
        self.assertEqual(len(result.results), 2)
        self.assertGreaterEqual(self.shoulder.stop_calls, 1)

    def test_group_timeout_marks_each_unfinished_axis_and_stops_them(self) -> None:
        self.shoulder.states = [joint_state(0.0)] * 20
        self.elbow.states = [joint_state(0.0)] * 20
        handle = self.controller.submit_positions(
            MultiAxisTarget(
                (
                    AxisTarget(AxisName.SHOULDER, 10.0, 2.0),
                    AxisTarget(AxisName.ELBOW, -10.0, 2.0),
                )
            )
        )
        result = self.controller.wait_group(handle, timeout_s=0.03)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.assertEqual(
            tuple(item.status for item in result.results),
            (MotionCommandStatus.TIMEOUT, MotionCommandStatus.TIMEOUT),
        )
        self.assertEqual(self.shoulder.stop_calls, 1)
        self.assertEqual(self.elbow.stop_calls, 1)

    def test_rotation_stop_is_explicitly_unsupported(self) -> None:
        result = self.controller.stop(AxisName.ROTATION)
        self.assertEqual(result.status, MotionCommandStatus.REJECTED)
        self.assertEqual(result.error_code, MotionErrorCode.UNSUPPORTED_COMMAND)
        self.assertEqual(self.rotation.disable_calls, 0)


if __name__ == "__main__":
    unittest.main()
