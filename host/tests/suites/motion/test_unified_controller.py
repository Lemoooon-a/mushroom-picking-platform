"""五轴统一异步点到点控制器的纯 fake 测试。"""

from __future__ import annotations

import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from config.project.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.project.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from drivers.stm32_motion import AxisStatus, STM32CommandSubmission, STM32Message
from drivers.stm32_motion import STM32MotionTimeoutError
from motion.authorization import MotionAuthorization, RuntimeMode
from motion.unified_controller import (
    MultiAxisSubmissionError,
    UnifiedMotionController,
    UnifiedMotionError,
)
from motion.unified_protocol import (
    ArrivalConfig,
    AxisName,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisTarget,
    RelativeAxisTarget,
)
from motion.suction import SuctionMode, SuctionStatus


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
        self.enabled = True
        self.enable_calls = 0
        self.disable_calls = 0

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

    def is_enabled(self) -> bool:
        return self.enabled

    def enable(self) -> None:
        self.enable_calls += 1
        self.enabled = True

    def disable(self) -> None:
        self.disable_calls += 1
        self.enabled = False

    def is_moving(self) -> bool:
        return bool(self.get_state().moving)


class FakeRotation:
    def __init__(self) -> None:
        self.config = END_EFFECTOR_ROTATION_CONFIG
        self.commands: list[tuple[float, int]] = []
        self.feedback: list[object] = []
        self.enable_calls = 0
        self.disable_calls = 0
        self.enabled = True

    def command_position(self, position_rad: float, speed_raw: int) -> int:
        self.commands.append((position_rad, speed_raw))
        return 123

    def read_feedback(self) -> object:
        if self.feedback:
            return self.feedback.pop(0)
        return rotation_feedback(0.0)

    def enable_torque(self) -> None:
        self.enable_calls += 1
        self.enabled = True

    def disable_torque(self) -> None:
        self.disable_calls += 1
        self.enabled = False

    def torque_enabled(self) -> bool:
        return self.enabled


class FakeSuction:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def _status(self, mode: SuctionMode) -> SuctionStatus:
        return SuctionStatus(
            mode=mode,
            command_acknowledged=True,
            physically_verified=False,
            vacuum_detected=None,
            pump_on=mode is SuctionMode.GRIP,
            release_open=mode is SuctionMode.RELEASE,
            busy=False,
            fault=0,
            raw_state={SuctionMode.IDLE: 0, SuctionMode.GRIP: 1, SuctionMode.RELEASE: 3}[mode],
        )

    def grip(self) -> SuctionStatus:
        self.calls.append("grip")
        return self._status(SuctionMode.GRIP)

    def release(self) -> SuctionStatus:
        self.calls.append("release")
        return self._status(SuctionMode.RELEASE)

    def idle(self) -> SuctionStatus:
        self.calls.append("idle")
        return self._status(SuctionMode.IDLE)

    def get_status(self) -> SuctionStatus:
        self.calls.append("status")
        return self._status(SuctionMode.IDLE)


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


def linear_position_limits() -> dict[AxisName, tuple[float, float]]:
    return {
        AxisName.SLIDE: (0.0, 799.988),
        AxisName.Z: (0.0, 190.0),
    }


def linear_motion_limits() -> dict[AxisName, tuple[float, float]]:
    return {
        AxisName.SLIDE: (72.0, 180.0),
        AxisName.Z: (10.0, 25.0),
    }


def motion_authorization() -> MotionAuthorization:
    return MotionAuthorization(
        mode=RuntimeMode.MOTION,
        allow_unverified_rotation_motion=True,
    )


class ControllerTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = FakeClock()
        self.stm32 = FakeSTM32()
        self.shoulder = FakeJoint(SHOULDER_JOINT_CONFIG)
        self.elbow = FakeJoint(ELBOW_JOINT_CONFIG)
        self.rotation = FakeRotation()
        self.suction = FakeSuction()
        self.controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            default_motion_parameters={
                AxisName.SLIDE: (2.0, 4.0),
                AxisName.Z: (1.0, 3.0),
                AxisName.SHOULDER: (5.0, None),
                AxisName.ELBOW: (6.0, None),
            },
            authorization=motion_authorization(),
            clock=self.clock,
            sleep=self.clock.advance,
            suction=self.suction,
        )

    def submit_home_without_wait(self, axis: AxisName) -> MotionCommandHandle:
        with patch.object(
            self.controller,
            "wait",
            side_effect=lambda handle, **_kwargs: handle,
        ):
            handle = self.controller.home_reference(axis)
        self.assertIsInstance(handle, MotionCommandHandle)
        return handle  # type: ignore[return-value]


class RelativeAxisMotionTests(ControllerTestCase):
    def test_all_five_axes_resolve_relative_delta_to_absolute_target(self) -> None:
        cases = (
            (AxisName.SLIDE, 10.0, 2.0, 12.0),
            (AxisName.Z, 50.0, -5.0, 45.0),
            (AxisName.SHOULDER, 10.0, 2.0, 12.0),
            (AxisName.ELBOW, -10.0, -2.0, -12.0),
            (AxisName.ROTATION, 20.0, 3.0, 23.0),
        )
        for axis, start, delta, expected in cases:
            with self.subTest(axis=axis.value):
                if axis in (AxisName.SLIDE, AxisName.Z):
                    self.stm32.states.append(
                        axis_status(
                            axis.value,
                            position_um=round(start * 1000),
                            busy=False,
                        )
                    )
                elif axis is AxisName.SHOULDER:
                    self.shoulder.states.append(joint_state(start))
                elif axis is AxisName.ELBOW:
                    self.elbow.states.append(joint_state(start))
                else:
                    self.rotation.feedback.append(rotation_feedback(start))
                handle = self.controller.submit_relative(
                    RelativeAxisTarget(axis, delta)
                )
                self.assertAlmostEqual(handle.target_position, expected)

    def test_relative_velocity_and_acceleration_reuse_absolute_dispatch(self) -> None:
        self.stm32.states.append(
            axis_status("slide", position_um=10_000, busy=False)
        )
        handle = self.controller.submit_relative(
            RelativeAxisTarget(AxisName.SLIDE, 5.0, 3.0, 6.0)
        )
        self.assertEqual(handle.target_position, 15.0)
        self.assertEqual(self.stm32.submissions[-1], ("slide", 15_000, 3_000, 6_000))

    def test_zero_delta_returns_arrived_without_hardware_submission(self) -> None:
        self.stm32.states.append(
            axis_status("slide", position_um=10_000, busy=False)
        )
        handle = self.controller.submit_relative(
            RelativeAxisTarget(AxisName.SLIDE, 0.0)
        )
        result = self.controller.wait(handle)
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)
        self.assertEqual(result.target_position, 10.0)
        self.assertIn("no motion submitted", result.message)
        self.assertEqual(self.stm32.submissions, [])

    def test_relative_preconditions_reject_before_hardware_submission(self) -> None:
        cases = (
            (axis_status("slide", position_um=0, busy=True), MotionErrorCode.BUSY),
            (axis_status("slide", position_um=0, busy=False, fault=1), MotionErrorCode.DEVICE_FAULT),
            (axis_status("slide", position_um=0, busy=False, valid=False), MotionErrorCode.POSITION_INVALID),
            (axis_status("slide", position_um=0, busy=False, homed=False), MotionErrorCode.NOT_HOMED),
        )
        for state, code in cases:
            with self.subTest(code=code.value):
                self.stm32.states.append(state)
                with self.assertRaises(UnifiedMotionError) as raised:
                    self.controller.submit_relative(
                        RelativeAxisTarget(AxisName.SLIDE, 1.0)
                    )
                self.assertEqual(raised.exception.error_code, code)
                self.assertEqual(self.stm32.submissions, [])

    def test_relative_soft_limits_reject_both_directions(self) -> None:
        for start, delta in ((799.0, 2.0), (1.0, -2.0)):
            with self.subTest(start=start, delta=delta):
                self.stm32.states.append(
                    axis_status(
                        "slide", position_um=round(start * 1000), busy=False
                    )
                )
                with self.assertRaises(UnifiedMotionError) as raised:
                    self.controller.submit_relative(
                        RelativeAxisTarget(AxisName.SLIDE, delta)
                    )
                self.assertEqual(raised.exception.error_code, MotionErrorCode.SOFT_LIMIT)
                self.assertEqual(self.stm32.submissions, [])

    def test_concurrent_relative_requests_cannot_both_use_same_start(self) -> None:
        self.stm32.states.extend(
            [axis_status("slide", position_um=10_000, busy=False)] * 2
        )
        barrier = threading.Barrier(2)
        outcomes: list[object] = []

        def submit() -> None:
            barrier.wait()
            try:
                outcomes.append(
                    self.controller.submit_relative(
                        RelativeAxisTarget(AxisName.SLIDE, 1.0)
                    )
                )
            except Exception as exc:
                outcomes.append(exc)

        threads = [threading.Thread(target=submit) for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=1.0)
        self.assertEqual(sum(isinstance(item, MotionCommandHandle) for item in outcomes), 1)
        self.assertEqual(sum(isinstance(item, UnifiedMotionError) for item in outcomes), 1)
        self.assertEqual(len(self.stm32.submissions), 1)


class SuctionAndRotaryLifecycleTests(ControllerTestCase):
    def test_suction_forwards_without_changing_joint_enable_state(self) -> None:
        before = (
            self.shoulder.enable_calls,
            self.shoulder.disable_calls,
            self.elbow.enable_calls,
            self.elbow.disable_calls,
            self.rotation.enable_calls,
            self.rotation.disable_calls,
        )
        self.assertEqual(self.controller.suction_grip().mode, SuctionMode.GRIP)
        self.assertEqual(self.controller.suction_release().mode, SuctionMode.RELEASE)
        self.assertEqual(self.controller.suction_idle().mode, SuctionMode.IDLE)
        self.assertEqual(self.controller.get_suction_status().mode, SuctionMode.IDLE)
        self.assertEqual(self.suction.calls, ["grip", "release", "idle", "status"])
        after = (
            self.shoulder.enable_calls,
            self.shoulder.disable_calls,
            self.elbow.enable_calls,
            self.elbow.disable_calls,
            self.rotation.enable_calls,
            self.rotation.disable_calls,
        )
        self.assertEqual(after, before)

    def test_missing_suction_capability_has_explicit_error(self) -> None:
        controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            authorization=motion_authorization(),
        )
        with self.assertRaisesRegex(UnifiedMotionError, "capability is unavailable"):
            controller.get_suction_status()

    def test_enable_group_uses_fixed_order_reloads_positions_and_is_idempotent(self) -> None:
        self.shoulder.enabled = False
        self.elbow.enabled = False
        self.rotation.enabled = False
        events: list[str] = []

        def shoulder_enable() -> None:
            events.append("enable:shoulder")
            self.shoulder.enabled = True

        def elbow_enable() -> None:
            events.append("enable:elbow")
            self.elbow.enabled = True

        def rotation_enable() -> None:
            events.append("enable:rotation")
            self.rotation.enabled = True

        with patch.object(self.shoulder, "enable", side_effect=shoulder_enable), patch.object(
            self.elbow, "enable", side_effect=elbow_enable
        ), patch.object(self.rotation, "enable_torque", side_effect=rotation_enable):
            status = self.controller.enable_rotary_joints()
            status_again = self.controller.enable_rotary_joints()

        self.assertTrue(status.all_enabled)
        self.assertTrue(status_again.all_enabled)
        self.assertEqual(
            events,
            ["enable:shoulder", "enable:elbow", "enable:rotation"],
        )
        self.assertEqual(self.shoulder.initialize_calls, 2)
        self.assertEqual(self.elbow.initialize_calls, 2)
        self.assertEqual(len(self.rotation.commands), 1)

    def test_partial_enable_failure_rolls_back_newly_enabled_joint(self) -> None:
        self.shoulder.enabled = False
        self.elbow.enabled = False
        self.rotation.enabled = False
        events: list[str] = []

        def shoulder_enable() -> None:
            events.append("enable:shoulder")
            self.shoulder.enabled = True

        def shoulder_disable() -> None:
            events.append("disable:shoulder")
            self.shoulder.enabled = False

        with patch.object(self.shoulder, "enable", side_effect=shoulder_enable), patch.object(
            self.shoulder, "disable", side_effect=shoulder_disable
        ), patch.object(self.elbow, "enable", side_effect=RuntimeError("elbow failed")):
            with self.assertRaisesRegex(UnifiedMotionError, "rollback attempted"):
                self.controller.enable_rotary_joints()
        self.assertEqual(events, ["enable:shoulder", "disable:shoulder"])
        self.assertFalse(self.shoulder.enabled)
        self.assertEqual(self.rotation.enable_calls, 0)

    def test_disable_stops_moving_can_joint_then_uses_reverse_order(self) -> None:
        self.shoulder.states = [
            joint_state(0.0, moving=True),
            joint_state(0.0, moving=False),
        ]
        events: list[str] = []

        def disable_rotation() -> None:
            events.append("disable:rotation")
            self.rotation.enabled = False

        def disable_elbow() -> None:
            events.append("disable:elbow")
            self.elbow.enabled = False

        def disable_shoulder() -> None:
            events.append("disable:shoulder")
            self.shoulder.enabled = False

        with patch.object(
            self.rotation, "disable_torque", side_effect=disable_rotation
        ), patch.object(self.elbow, "disable", side_effect=disable_elbow), patch.object(
            self.shoulder, "disable", side_effect=disable_shoulder
        ):
            status = self.controller.disable_rotary_joints()
        self.assertEqual(self.shoulder.stop_calls, 1)
        self.assertEqual(
            events,
            ["disable:rotation", "disable:elbow", "disable:shoulder"],
        )
        self.assertFalse(status.all_enabled)

    def test_disable_stops_and_confirms_moving_linear_axis_first(self) -> None:
        self.stm32.states = [
            axis_status("z", position_um=0, busy=True),
            axis_status("slide", position_um=0, busy=False),
            axis_status("z", position_um=0, busy=False),
        ]
        status = self.controller.disable_rotary_joints()
        self.assertEqual(self.stm32.stop_calls, ["z"])
        self.assertFalse(status.all_enabled)

    def test_moving_rotation_refuses_disable_without_removing_torque(self) -> None:
        self.rotation.feedback = [rotation_feedback(0.0, moving=True)]
        with self.assertRaisesRegex(UnifiedMotionError, "no verified independent stop"):
            self.controller.disable_rotary_joints()
        self.assertEqual(self.rotation.disable_calls, 0)
        self.assertEqual(self.shoulder.disable_calls, 0)
        self.assertEqual(self.elbow.disable_calls, 0)

    def test_repeated_disable_is_idempotent(self) -> None:
        first = self.controller.disable_rotary_joints()
        calls = (
            self.shoulder.disable_calls,
            self.elbow.disable_calls,
            self.rotation.disable_calls,
        )
        second = self.controller.disable_rotary_joints()
        self.assertEqual(first, second)
        self.assertEqual(
            (
                self.shoulder.disable_calls,
                self.elbow.disable_calls,
                self.rotation.disable_calls,
            ),
            calls,
        )

    def test_disabled_group_rejects_rotary_motion_until_explicit_enable(self) -> None:
        self.shoulder.enabled = False
        with self.assertRaisesRegex(
            UnifiedMotionError,
            'Run "joints enable" before motion',
        ):
            self.controller.submit_absolute(
                AxisTarget(AxisName.SHOULDER, 1.0, 2.0)
            )
        self.assertEqual(self.shoulder.commands, [])

    def test_stop_never_disables_holding_torque(self) -> None:
        self.controller.stop(AxisName.SHOULDER)
        self.assertEqual(self.shoulder.stop_calls, 1)
        self.assertEqual(self.shoulder.disable_calls, 0)
        self.assertEqual(self.elbow.disable_calls, 0)
        self.assertEqual(self.rotation.disable_calls, 0)
        self.assertTrue(self.controller.rotary_joints_enabled())


class DescriptorAndDispatchTests(ControllerTestCase):
    def test_lists_five_axes_with_public_units(self) -> None:
        descriptors = self.controller.list_axes()
        self.assertEqual(tuple(item.name for item in descriptors), tuple(AxisName))
        self.assertEqual(descriptors[0].position_unit, "mm")
        self.assertEqual(descriptors[2].position_unit, "deg")
        self.assertFalse(descriptors[-1].capabilities.stop)

    def test_linear_position_limits_are_required_valid_constructor_data(self) -> None:
        invalid_limits = (
            {},
            {AxisName.SLIDE: (0.0, 1.0)},
            {AxisName.SLIDE: (0.0, 1.0), AxisName.Z: (0.0, 1.0, 2.0)},
            {AxisName.SLIDE: (0.0, 1.0), AxisName.Z: (1.0, 1.0)},
            {AxisName.SLIDE: (0.0, 1.0), AxisName.Z: (0.0, math.inf)},
        )
        for limits in invalid_limits:
            with self.subTest(limits=limits):
                with self.assertRaises((TypeError, ValueError)):
                    UnifiedMotionController(
                        stm32_client=self.stm32,
                        shoulder_joint=self.shoulder,
                        elbow_joint=self.elbow,
                        rotation_axis=self.rotation,
                        linear_position_limits=limits,  # type: ignore[arg-type]
                        linear_motion_limits=linear_motion_limits(),
                        arrival_configs=arrival_configs(),
                        authorization=motion_authorization(),
                    )

    def test_linear_motion_limits_are_required_valid_constructor_data(self) -> None:
        invalid_limits = (
            {},
            {AxisName.SLIDE: (72.0, 180.0)},
            {AxisName.SLIDE: (72.0, 180.0), AxisName.Z: (10.0,)},
            {AxisName.SLIDE: (0.0, 180.0), AxisName.Z: (10.0, 25.0)},
            {AxisName.SLIDE: (72.0, 180.0), AxisName.Z: (10.0, math.inf)},
        )
        for limits in invalid_limits:
            with self.subTest(limits=limits):
                with self.assertRaises((TypeError, ValueError)):
                    UnifiedMotionController(
                        stm32_client=self.stm32,
                        shoulder_joint=self.shoulder,
                        elbow_joint=self.elbow,
                        rotation_axis=self.rotation,
                        linear_position_limits=linear_position_limits(),
                        linear_motion_limits=limits,  # type: ignore[arg-type]
                        arrival_configs=arrival_configs(),
                        authorization=motion_authorization(),
                    )

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
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            default_motion_parameters={AxisName.Z: (1.0, 2.0)},
            authorization=motion_authorization(),
            clock=self.clock,
            sleep=self.clock.advance,
        )
        z_controller.submit_absolute(AxisTarget(AxisName.Z, 1.0))
        self.assertEqual(self.stm32.submissions[-1][1], 1000)

    def test_linear_motion_limits_accept_equal_and_reject_excess_without_io(self) -> None:
        self.controller.submit_absolute(
            AxisTarget(AxisName.SLIDE, 1.0, 72.0, 180.0)
        )
        self.assertEqual(self.stm32.submissions[-1][2:], (72000, 180000))

        for axis, velocity, acceleration, expected in (
            (AxisName.SLIDE, 72.001, 180.0, "velocity"),
            (AxisName.SLIDE, 72.0, 180.001, "acceleration"),
            (AxisName.Z, 10.001, 25.0, "velocity"),
            (AxisName.Z, 10.0, 25.001, "acceleration"),
        ):
            before = len(self.stm32.submissions)
            with self.subTest(axis=axis.value, expected=expected):
                with self.assertRaises(UnifiedMotionError) as failure:
                    self.controller.submit_absolute(
                        AxisTarget(axis, 1.0, velocity, acceleration)
                    )
                self.assertEqual(
                    failure.exception.error_code,
                    MotionErrorCode.SOFT_LIMIT,
                )
                self.assertIn(expected, str(failure.exception))
                self.assertIn(axis.value, str(failure.exception))
                self.assertEqual(len(self.stm32.submissions), before)

    def test_default_linear_motion_limits_are_enforced_without_io(self) -> None:
        controller = UnifiedMotionController(
            stm32_client=self.stm32,
            shoulder_joint=self.shoulder,
            elbow_joint=self.elbow,
            rotation_axis=self.rotation,
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            default_motion_parameters={AxisName.Z: (10.001, 25.0)},
            authorization=motion_authorization(),
        )
        with self.assertRaises(UnifiedMotionError) as failure:
            controller.submit_absolute(AxisTarget(AxisName.Z, 1.0))
        self.assertEqual(failure.exception.error_code, MotionErrorCode.SOFT_LIMIT)
        self.assertEqual(self.stm32.submissions, [])

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
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            default_motion_parameters={AxisName.SLIDE: (1.0, 1.0)},
            authorization=motion_authorization(),
        )
        with self.assertRaises(UnifiedMotionError) as missing:
            controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        self.assertEqual(missing.exception.error_code, MotionErrorCode.BACKEND_UNAVAILABLE)
        with self.assertRaises(UnifiedMotionError) as unknown:
            self.controller.describe_axis("bogus")  # type: ignore[arg-type]
        self.assertEqual(unknown.exception.error_code, MotionErrorCode.UNKNOWN_AXIS)
        rotation_limit = self.controller.describe_axis(
            AxisName.ROTATION
        ).maximum_position
        with self.assertRaises(UnifiedMotionError) as limit:
            self.controller.submit_absolute(
                AxisTarget(AxisName.ROTATION, rotation_limit + 1.0)
            )
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
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(),
            authorization=motion_authorization(),
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
        self.assertIn("no verified independent stop", result.message)
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
                    linear_position_limits=linear_position_limits(),
                    linear_motion_limits=linear_motion_limits(),
                    arrival_configs=arrival_configs(),
                    default_motion_parameters={AxisName.SLIDE: (1.0, 2.0)},
                    authorization=motion_authorization(),
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


class STM32HomingStateTests(ControllerTestCase):
    def test_slide_and_z_position_invalid_are_expected_home_transients(self) -> None:
        for axis in (AxisName.SLIDE, AxisName.Z):
            with self.subTest(axis=axis.value):
                self.stm32.states.append(
                    axis_status(
                        axis.value,
                        position_um=0,
                        busy=True,
                        homed=False,
                        valid=False,
                        fault=2,
                    )
                )
                handle = self.submit_home_without_wait(axis)
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.MOVING)
                self.assertTrue(result.accepted)
                self.assertIsNone(result.completed)
        self.assertEqual(self.stm32.stop_calls, [])

    def test_repeated_position_invalid_home_polls_do_not_resubmit_or_stop(self) -> None:
        self.stm32.states = [
            axis_status(
                "slide",
                position_um=0,
                busy=busy,
                homed=False,
                valid=False,
                fault=2,
            )
            for busy in (True, True, False)
        ]
        handle = self.submit_home_without_wait(AxisName.SLIDE)
        statuses = tuple(
            self.controller.get_command_result(handle).status for _ in range(3)
        )
        self.assertEqual(
            statuses,
            (
                MotionCommandStatus.MOVING,
                MotionCommandStatus.MOVING,
                MotionCommandStatus.ACCEPTED,
            ),
        )
        self.assertEqual(self.stm32.home_calls, ["slide"])
        self.assertEqual(self.stm32.stop_calls, [])

    def test_position_invalid_transients_then_done_reaches_arrived(self) -> None:
        self.stm32.events = [
            None,
            None,
            STM32Message("!", 0, "DONE", ("S", "0")),
        ]
        self.stm32.states = [
            axis_status(
                "slide",
                position_um=0,
                busy=True,
                homed=False,
                valid=False,
                fault=2,
            ),
            axis_status(
                "slide",
                position_um=0,
                busy=True,
                homed=False,
                valid=False,
                fault=2,
            ),
            axis_status("slide", position_um=0, busy=False),
        ]
        handle = self.submit_home_without_wait(AxisName.SLIDE)
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        self.assertEqual(
            self.controller.get_command_result(handle).status,
            MotionCommandStatus.MOVING,
        )
        result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.ARRIVED)
        self.assertTrue(result.completed)

    def test_abort_and_fault_events_take_priority_after_transient(self) -> None:
        for kind, expected in (
            ("ABORT", MotionCommandStatus.ABORTED),
            ("FAULT", MotionCommandStatus.FAULT),
        ):
            with self.subTest(kind=kind):
                self.stm32.events = [
                    None,
                    STM32Message("!", self.stm32.sequence, kind, ("Z",)),
                ]
                self.stm32.states = [
                    axis_status(
                        "z",
                        position_um=0,
                        busy=True,
                        homed=False,
                        valid=False,
                        fault=2,
                    )
                ]
                handle = self.submit_home_without_wait(AxisName.Z)
                self.assertEqual(
                    self.controller.get_command_result(handle).status,
                    MotionCommandStatus.MOVING,
                )
                self.assertEqual(
                    self.controller.get_command_result(handle).status,
                    expected,
                )

    def test_real_and_unknown_home_faults_fail_with_stable_semantics(self) -> None:
        for fault, name in (
            (1, "stm32_axis.limit"),
            (3, "stm32_axis.hardware_or_config"),
            (4, "stm32_axis.homing"),
            (9, "stm32_axis.unknown"),
        ):
            with self.subTest(fault=fault):
                self.stm32.states = [
                    axis_status(
                        "slide",
                        position_um=0,
                        busy=True,
                        homed=False,
                        valid=False,
                        fault=fault,
                    )
                ]
                handle = self.submit_home_without_wait(AxisName.SLIDE)
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.FAULT)
                self.assertEqual(result.error_code, MotionErrorCode.DEVICE_FAULT)
                self.assertIn(name, result.message)
                self.assertIn(str(fault), result.message)

    def test_position_invalid_is_transient_only_with_exact_home_state(self) -> None:
        for homed, valid in ((True, False), (False, True), (True, True)):
            with self.subTest(homed=homed, valid=valid):
                self.stm32.states = [
                    axis_status(
                        "z",
                        position_um=0,
                        busy=True,
                        homed=homed,
                        valid=valid,
                        fault=2,
                    )
                ]
                handle = self.submit_home_without_wait(AxisName.Z)
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.FAULT)
                self.assertEqual(result.error_code, MotionErrorCode.POSITION_INVALID)

    def test_done_requires_all_home_postconditions(self) -> None:
        cases = (
            (
                axis_status("slide", position_um=0, busy=False, homed=False),
                MotionErrorCode.POSITION_INVALID,
                "homed is not true",
            ),
            (
                axis_status("slide", position_um=0, busy=False, valid=False),
                MotionErrorCode.POSITION_INVALID,
                "position remains invalid",
            ),
            (
                axis_status("slide", position_um=0, busy=True),
                MotionErrorCode.BACKEND_ERROR,
                "still busy",
            ),
            (
                axis_status(
                    "slide",
                    position_um=0,
                    busy=False,
                    homed=False,
                    valid=False,
                    fault=2,
                ),
                MotionErrorCode.POSITION_INVALID,
                "stm32_axis.position_invalid",
            ),
        )
        for state, error_code, message in cases:
            with self.subTest(error_code=error_code, message=message):
                self.stm32.events = [
                    STM32Message("!", self.stm32.sequence, "DONE", ("S", "0"))
                ]
                self.stm32.states = [state]
                handle = self.submit_home_without_wait(AxisName.SLIDE)
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.FAULT)
                self.assertEqual(result.error_code, error_code)
                self.assertIn(message, result.message)

    def test_done_real_faults_remain_device_faults(self) -> None:
        for fault, name in (
            (1, "stm32_axis.limit"),
            (3, "stm32_axis.hardware_or_config"),
            (4, "stm32_axis.homing"),
        ):
            with self.subTest(fault=fault):
                self.stm32.events = [
                    STM32Message("!", self.stm32.sequence, "DONE", ("Z", "0"))
                ]
                self.stm32.states = [
                    axis_status("z", position_um=0, busy=False, fault=fault)
                ]
                handle = self.submit_home_without_wait(AxisName.Z)
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.FAULT)
                self.assertEqual(result.error_code, MotionErrorCode.DEVICE_FAULT)
                self.assertIn(name, result.message)

    def test_done_state_query_failure_is_communication_error(self) -> None:
        self.stm32.events = [STM32Message("!", 0, "DONE", ("Z", "0"))]
        handle = self.submit_home_without_wait(AxisName.Z)
        with patch.object(
            self.stm32,
            "query_axis",
            side_effect=STM32MotionTimeoutError("query timed out"),
        ):
            result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.COMMUNICATION_ERROR)
        self.assertEqual(result.error_code, MotionErrorCode.COMMUNICATION_ERROR)

    def test_position_invalid_remains_fault_for_absolute_move(self) -> None:
        self.stm32.states = [
            axis_status(
                "slide",
                position_um=0,
                busy=True,
                homed=False,
                valid=False,
                fault=2,
            )
        ]
        handle = self.controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        result = self.controller.get_command_result(handle)
        self.assertEqual(result.status, MotionCommandStatus.FAULT)
        self.assertEqual(result.error_code, MotionErrorCode.POSITION_INVALID)
        self.assertIn("move_absolute", result.message)

    def test_other_faults_remain_device_faults_for_absolute_move(self) -> None:
        for fault, name in (
            (1, "stm32_axis.limit"),
            (3, "stm32_axis.hardware_or_config"),
            (4, "stm32_axis.homing"),
            (9, "stm32_axis.unknown"),
        ):
            with self.subTest(fault=fault):
                self.stm32.states = [
                    axis_status(
                        "slide",
                        position_um=0,
                        busy=True,
                        homed=False,
                        valid=False,
                        fault=fault,
                    )
                ]
                handle = self.controller.submit_absolute(
                    AxisTarget(AxisName.SLIDE, 1.0)
                )
                result = self.controller.get_command_result(handle)
                self.assertEqual(result.status, MotionCommandStatus.FAULT)
                self.assertEqual(result.error_code, MotionErrorCode.DEVICE_FAULT)
                self.assertIn(name, result.message)

    def test_home_position_invalid_timeout_stops_once(self) -> None:
        self.stm32.states = [
            axis_status(
                "z",
                position_um=0,
                busy=True,
                homed=False,
                valid=False,
                fault=2,
            )
        ] * 20
        result = self.controller.home_reference(AxisName.Z, timeout_s=0.03)
        self.assertEqual(result.status, MotionCommandStatus.TIMEOUT)
        self.assertEqual(self.stm32.stop_calls, ["z"])
        self.assertIn("best-effort software stop", result.message)
        self.assertIn("not an emergency stop", result.message)


class MultiAxisTests(ControllerTestCase):
    def test_group_z_overspeed_is_rejected_before_any_backend_write(self) -> None:
        target = MultiAxisTarget(
            (
                AxisTarget(AxisName.SLIDE, 2.0, 60.0, 180.0),
                AxisTarget(AxisName.Z, 2.0, 10.001, 25.0),
                AxisTarget(AxisName.SHOULDER, 2.0, 10.0),
                AxisTarget(AxisName.ELBOW, -2.0, 10.0),
                AxisTarget(AxisName.ROTATION, 2.0),
            )
        )

        with self.assertRaises(UnifiedMotionError) as failure:
            self.controller.submit_positions(target)

        self.assertEqual(failure.exception.error_code, MotionErrorCode.SOFT_LIMIT)
        self.assertEqual(failure.exception.axis, AxisName.Z)
        self.assertEqual(self.stm32.submissions, [])
        self.assertEqual(self.shoulder.commands, [])
        self.assertEqual(self.elbow.commands, [])
        self.assertEqual(self.rotation.commands, [])

    def test_public_group_validation_has_no_control_io(self) -> None:
        target = MultiAxisTarget(
            (
                AxisTarget(AxisName.SLIDE, 2.0, 60.0, 180.0),
                AxisTarget(AxisName.Z, 2.0, 8.0, 25.0),
                AxisTarget(AxisName.SHOULDER, 2.0, 10.0),
                AxisTarget(AxisName.ELBOW, -2.0, 10.0),
                AxisTarget(AxisName.ROTATION, 2.0),
            )
        )

        self.controller.validate_positions(target)

        self.assertEqual(self.stm32.submissions, [])
        self.assertEqual(self.shoulder.commands, [])
        self.assertEqual(self.elbow.commands, [])
        self.assertEqual(self.rotation.commands, [])

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
            linear_position_limits=linear_position_limits(),
            linear_motion_limits=linear_motion_limits(),
            arrival_configs=arrival_configs(stable_time_s=0.0),
            default_motion_parameters={
                AxisName.SHOULDER: (2.0, None),
                AxisName.ELBOW: (2.0, None),
            },
            authorization=motion_authorization(),
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
