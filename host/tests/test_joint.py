"""有限行程 MG4010 旋转关节的离线测试。"""

from __future__ import annotations

from contextlib import redirect_stdout
import io
import math
import unittest
from unittest.mock import MagicMock, patch

import can

from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from drivers.mg4010_protocol import MotorFault, MotorSingleTurnPosition, MotorStatus
from robot.joint import (
    CanRotaryJoint,
    JointConfig,
    JointConfigurationError,
    JointInitializationError,
    JointLimitError,
    JointMotorDisabledError,
    JointMotorFaultError,
    JointMotorMovingError,
    JointPositionOutOfRangeError,
    JointState,
    joint_position_to_output_abs_deg,
    resolve_output_angle_to_joint_position,
    wrap_360,
)
from scripts import test_joint_position as joint_cli


def make_config(**overrides: object) -> JointConfig:
    values: dict[str, object] = {
        "name": "shoulder",
        "motor_id": 1,
        "gear_ratio": 36.0,
        "direction_sign": 1,
        "encoder_zero_output_deg": 350.0,
        "min_position_rad": math.radians(-20),
        "max_position_rad": math.radians(40),
        "max_velocity_rad_s": math.radians(10),
        "position_tolerance_rad": math.radians(0.1),
        "moving_velocity_threshold_rad_s": math.radians(0.05),
    }
    values.update(overrides)
    return JointConfig(**values)  # type: ignore[arg-type]


def single_for_output(output_deg: float, ratio: float = 36.0) -> MotorSingleTurnPosition:
    motor_deg = (output_deg % 360.0) * ratio
    return MotorSingleTurnPosition(
        circle_angle_raw=round(motor_deg * 100),
        motor_cycle_deg=motor_deg,
    )


class FakeDriver:
    def __init__(
        self,
        *,
        motor_id: int = 1,
        single_samples: list[MotorSingleTurnPosition] | None = None,
        multi_turn_deg: float = 0.0,
        speed_deg_s: int = 0,
        motor_state: int = 0,
        error_state: int = 0,
    ) -> None:
        self.motor_id = motor_id
        self.single_samples = single_samples or [single_for_output(350.0)]
        self.last_single = self.single_samples[-1]
        self.multi_turn_deg = multi_turn_deg
        self.status = MotorStatus(
            temperature_c=30,
            torque_current_raw=0,
            torque_current_a=0.0,
            motor_speed_deg_s=speed_deg_s,
            encoder_raw=0,
        )
        self.fault = MotorFault(
            temperature_c=30,
            bus_voltage_v=24.0,
            bus_current_a=0.0,
            motor_state=motor_state,
            error_state=error_state,
        )
        self.commands: list[tuple[float, float]] = []
        self.stop_calls = 0
        self.multi_reads = 0

    def read_single_turn_position(self) -> MotorSingleTurnPosition:
        if self.single_samples:
            self.last_single = self.single_samples.pop(0)
        return self.last_single

    def read_multi_turn_position_deg(self) -> float:
        self.multi_reads += 1
        return self.multi_turn_deg

    def read_status(self) -> MotorStatus:
        return self.status

    def read_fault(self) -> MotorFault:
        return self.fault

    def command_position(
        self, *, target_motor_deg: float, max_motor_speed_deg_s: float
    ) -> None:
        self.commands.append((target_motor_deg, max_motor_speed_deg_s))

    def stop(self) -> None:
        self.stop_calls += 1


def initialized_joint(driver: FakeDriver, config: JointConfig) -> CanRotaryJoint:
    initial_output = config.encoder_zero_output_deg % 360.0
    driver.single_samples = [
        single_for_output(initial_output, config.gear_ratio),
        single_for_output(initial_output, config.gear_ratio),
        single_for_output(initial_output, config.gear_ratio),
    ]
    driver.last_single = driver.single_samples[-1]
    joint = CanRotaryJoint(driver, config)  # type: ignore[arg-type]
    joint.initialize(sample_interval=0)
    return joint


class AngleResolutionTests(unittest.TestCase):
    def test_wrap_360(self) -> None:
        self.assertEqual(wrap_360(360.0), 0.0)
        self.assertEqual(wrap_360(-10.0), 350.0)

    def test_finite_range_without_encoder_wrap(self) -> None:
        config = make_config(
            encoder_zero_output_deg=100.0,
            min_position_rad=math.radians(-30),
            max_position_rad=math.radians(40),
        )
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(120.0, config),
            math.radians(20),
        )

    def test_crossing_encoder_zero(self) -> None:
        config = make_config()
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(5.0, config),
            math.radians(15),
        )

    def test_range_wider_than_pi(self) -> None:
        config = make_config(
            encoder_zero_output_deg=0.0,
            min_position_rad=math.radians(-170),
            max_position_rad=math.radians(189),
        )
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(185.0, config),
            math.radians(185),
        )

    def test_direction_sign_positive_and_negative(self) -> None:
        positive = make_config(
            encoder_zero_output_deg=100.0,
            min_position_rad=math.radians(-30),
            max_position_rad=math.radians(30),
        )
        negative = make_config(
            encoder_zero_output_deg=100.0,
            direction_sign=-1,
            min_position_rad=math.radians(-30),
            max_position_rad=math.radians(30),
        )
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(110.0, positive),
            math.radians(10),
        )
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(110.0, negative),
            math.radians(-10),
        )

    def test_no_candidate_raises_out_of_range(self) -> None:
        with self.assertRaises(JointPositionOutOfRangeError):
            resolve_output_angle_to_joint_position(180.0, make_config())

    def test_ambiguous_candidate_raises_configuration_error(self) -> None:
        config = make_config(
            encoder_zero_output_deg=0.0,
            min_position_rad=0.0,
            max_position_rad=math.tau - 5e-13,
        )
        with self.assertRaises(JointConfigurationError):
            resolve_output_angle_to_joint_position(0.0, config)

    def test_range_must_be_strictly_less_than_tau(self) -> None:
        with self.assertRaisesRegex(JointConfigurationError, "less than 2\\*pi"):
            make_config(min_position_rad=-math.pi, max_position_rad=math.pi)

    def test_software_zero_and_inverse_conversion(self) -> None:
        config = make_config()
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(350.0, config), 0.0
        )
        self.assertAlmostEqual(
            joint_position_to_output_abs_deg(math.radians(15), config), 5.0
        )


class ConfiguredJointTests(unittest.TestCase):
    def test_calibrated_identity_and_limits(self) -> None:
        self.assertEqual(SHOULDER_JOINT_CONFIG.motor_id, 1)
        self.assertEqual(ELBOW_JOINT_CONFIG.motor_id, 2)
        for config in (SHOULDER_JOINT_CONFIG, ELBOW_JOINT_CONFIG):
            self.assertEqual(config.gear_ratio, 36.0)
            self.assertAlmostEqual(config.max_velocity_rad_s, math.radians(50))
        self.assertEqual(SHOULDER_JOINT_CONFIG.encoder_zero_output_deg, 100.0)
        self.assertEqual(SHOULDER_JOINT_CONFIG.direction_sign, 1)
        self.assertAlmostEqual(
            SHOULDER_JOINT_CONFIG.min_position_rad, math.radians(-60)
        )
        self.assertAlmostEqual(
            SHOULDER_JOINT_CONFIG.max_position_rad, math.radians(70)
        )
        self.assertEqual(ELBOW_JOINT_CONFIG.encoder_zero_output_deg, 158.0)
        self.assertEqual(ELBOW_JOINT_CONFIG.direction_sign, -1)
        self.assertAlmostEqual(
            ELBOW_JOINT_CONFIG.min_position_rad, math.radians(-152)
        )
        self.assertAlmostEqual(
            ELBOW_JOINT_CONFIG.max_position_rad, math.radians(152)
        )

    def test_calibrated_velocity_limit_conversion(self) -> None:
        converted = joint_cli.joint_velocity_to_motor_speed_deg_s(
            0.5, SHOULDER_JOINT_CONFIG
        )
        self.assertAlmostEqual(converted, math.degrees(0.5) * 36.0)

    def test_shoulder_calibration_maps_measured_angles(self) -> None:
        config = SHOULDER_JOINT_CONFIG
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(40, config), math.radians(-60)
        )
        self.assertAlmostEqual(resolve_output_angle_to_joint_position(100, config), 0.0)
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(170, config), math.radians(70)
        )

    def test_reversed_elbow_calibration_crosses_encoder_wrap(self) -> None:
        config = ELBOW_JOINT_CONFIG
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(310, config), math.radians(-152)
        )
        self.assertAlmostEqual(resolve_output_angle_to_joint_position(158, config), 0.0)
        self.assertAlmostEqual(
            resolve_output_angle_to_joint_position(6, config), math.radians(152)
        )

    def test_calibrated_limits_reject_outside_output_angles(self) -> None:
        with self.assertRaises(JointPositionOutOfRangeError):
            resolve_output_angle_to_joint_position(180, SHOULDER_JOINT_CONFIG)
        with self.assertRaises(JointPositionOutOfRangeError):
            resolve_output_angle_to_joint_position(0, ELBOW_JOINT_CONFIG)


class InitializationTests(unittest.TestCase):
    def test_unconfigured_template_is_rejected(self) -> None:
        with self.assertRaisesRegex(JointConfigurationError, "fully calibrated"):
            CanRotaryJoint(FakeDriver(), None)  # type: ignore[arg-type]

    def test_initialize_ignores_unstable_sample(self) -> None:
        driver = FakeDriver(
            single_samples=[
                single_for_output(10.0),
                single_for_output(350.0),
                single_for_output(350.03),
                single_for_output(349.98),
            ]
        )
        state = CanRotaryJoint(driver, make_config()).initialize()  # type: ignore[arg-type]
        self.assertTrue(state.position_valid)
        self.assertAlmostEqual(state.position_rad, math.radians(-0.02), places=5)

    def test_real_zero_is_valid_stable_position(self) -> None:
        config = make_config(
            encoder_zero_output_deg=0.0,
            min_position_rad=math.radians(-20),
            max_position_rad=math.radians(20),
        )
        driver = FakeDriver(
            single_samples=[
                single_for_output(0.0),
                single_for_output(0.0),
                single_for_output(0.0),
            ]
        )
        state = CanRotaryJoint(driver, config).initialize()  # type: ignore[arg-type]
        self.assertEqual(state.circle_angle_raw, 0)
        self.assertAlmostEqual(state.position_rad, 0.0)

    def test_impossible_0x94_cycle_value_is_rejected(self) -> None:
        invalid = MotorSingleTurnPosition(
            circle_angle_raw=1_296_000,
            motor_cycle_deg=12_960.0,
        )
        driver = FakeDriver(single_samples=[invalid, invalid, invalid])
        with self.assertRaises(JointPositionOutOfRangeError):
            CanRotaryJoint(driver, make_config()).initialize(  # type: ignore[arg-type]
                sample_interval=0
            )

    def test_command_requires_initialize(self) -> None:
        joint = CanRotaryJoint(FakeDriver(), make_config())  # type: ignore[arg-type]
        with self.assertRaises(JointInitializationError):
            joint.command_position(math.radians(5), math.radians(1))


class CommandTests(unittest.TestCase):
    def test_target_boundaries_are_allowed(self) -> None:
        config = make_config()
        for target in (config.min_position_rad, config.max_position_rad):
            driver = FakeDriver(multi_turn_deg=100.0)
            joint = initialized_joint(driver, config)
            joint.command_position(target, math.radians(1))
            self.assertEqual(len(driver.commands), 1)

    def test_target_outside_limit_sends_nothing(self) -> None:
        driver = FakeDriver()
        joint = initialized_joint(driver, make_config())
        with self.assertRaises(JointLimitError):
            joint.command_position(math.radians(41), math.radians(1))
        self.assertEqual(driver.commands, [])

    def test_velocity_outside_limit_sends_nothing(self) -> None:
        driver = FakeDriver()
        joint = initialized_joint(driver, make_config())
        with self.assertRaises(JointLimitError):
            joint.command_position(math.radians(5), math.radians(11))
        self.assertEqual(driver.commands, [])

    def test_nan_and_infinity_are_rejected(self) -> None:
        driver = FakeDriver()
        joint = initialized_joint(driver, make_config())
        for target, velocity in (
            (math.nan, math.radians(1)),
            (math.inf, math.radians(1)),
            (0.0, math.nan),
            (0.0, math.inf),
        ):
            with self.assertRaises(JointLimitError):
                joint.command_position(target, velocity)
        self.assertEqual(driver.commands, [])

    def test_within_position_tolerance_sends_no_a4(self) -> None:
        driver = FakeDriver(multi_turn_deg=200.0)
        joint = initialized_joint(driver, make_config())
        joint.command_position(math.radians(0.05), math.radians(1))
        self.assertEqual(driver.commands, [])
        self.assertEqual(driver.multi_reads, 0)

    def test_current_0x94_defines_joint_position(self) -> None:
        driver = FakeDriver(multi_turn_deg=-3562.5)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [single_for_output(5.0)]
        state = joint.get_state()
        self.assertAlmostEqual(state.position_rad, math.radians(15))

    def test_example_one_dynamic_a4_target(self) -> None:
        config = make_config(
            encoder_zero_output_deg=100.0,
            min_position_rad=math.radians(-90),
            max_position_rad=math.radians(90),
        )
        driver = FakeDriver(multi_turn_deg=-3562.5)
        joint = initialized_joint(driver, config)
        driver.single_samples = [single_for_output(110.0)]
        joint.command_position(math.radians(30), math.radians(2))
        self.assertAlmostEqual(driver.commands[0][0], -2842.5)
        self.assertAlmostEqual(driver.commands[0][0] + 3562.5, 720.0)

    def test_example_two_negative_delta(self) -> None:
        config = make_config(
            encoder_zero_output_deg=100.0,
            min_position_rad=math.radians(-90),
            max_position_rad=math.radians(90),
        )
        driver = FakeDriver(multi_turn_deg=1080.0)
        joint = initialized_joint(driver, config)
        driver.single_samples = [single_for_output(130.0)]
        joint.command_position(math.radians(-10), math.radians(2))
        self.assertAlmostEqual(driver.commands[0][0], -360.0)

    def test_reboot_branch_changes_target_not_mechanical_delta(self) -> None:
        config = make_config(
            encoder_zero_output_deg=100.0,
            min_position_rad=math.radians(-90),
            max_position_rad=math.radians(90),
        )
        targets: list[float] = []
        for multi in (9397.45, -3562.55):
            driver = FakeDriver(multi_turn_deg=multi)
            joint = initialized_joint(driver, config)
            driver.single_samples = [single_for_output(100.0)]
            joint.command_position(math.radians(10), math.radians(2))
            target = driver.commands[0][0]
            self.assertAlmostEqual(target - multi, 360.0)
            targets.append(target)
        self.assertAlmostEqual(targets[0] - targets[1], 12960.0)

    def test_direction_sign_negative_reverses_motor_delta(self) -> None:
        config = make_config(direction_sign=-1)
        driver = FakeDriver(multi_turn_deg=500.0)
        joint = initialized_joint(driver, config)
        driver.single_samples = [single_for_output(350.0)]
        joint.command_position(math.radians(10), math.radians(1))
        self.assertAlmostEqual(driver.commands[0][0], 140.0)

    def test_fault_rejects_motion(self) -> None:
        driver = FakeDriver(error_state=0x40)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [single_for_output(350.0)]
        with self.assertRaisesRegex(JointMotorFaultError, "0x40"):
            joint.command_position(math.radians(5), math.radians(1))
        self.assertEqual(driver.commands, [])

    def test_moving_motor_rejects_new_position_command(self) -> None:
        driver = FakeDriver(speed_deg_s=72)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [single_for_output(350.0)]
        with self.assertRaises(JointMotorMovingError):
            joint.command_position(math.radians(5), math.radians(1))
        self.assertEqual(driver.multi_reads, 0)
        self.assertEqual(driver.commands, [])

    def test_changing_0x94_around_0x92_rejects_command(self) -> None:
        driver = FakeDriver(multi_turn_deg=100.0)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [
            single_for_output(350.0),
            single_for_output(351.0),
        ]
        with self.assertRaisesRegex(JointMotorMovingError, "0x94 changed"):
            joint.command_position(math.radians(5), math.radians(1))
        self.assertEqual(driver.multi_reads, 1)
        self.assertEqual(driver.commands, [])

    def test_disabled_or_unknown_motor_state_rejects_motion(self) -> None:
        for state in (0x10, 0x22):
            driver = FakeDriver(motor_state=state)
            joint = initialized_joint(driver, make_config())
            driver.single_samples = [single_for_output(350.0)]
            with self.assertRaises(JointMotorDisabledError):
                joint.command_position(math.radians(5), math.radians(1))
            self.assertEqual(driver.commands, [])

    def test_command_is_non_blocking_with_respect_to_motion(self) -> None:
        driver = FakeDriver(multi_turn_deg=0.0)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [single_for_output(350.0)]
        returned = joint.command_position(math.radians(5), math.radians(1))
        self.assertEqual(len(driver.commands), 1)
        self.assertAlmostEqual(returned.position_rad, 0.0)

    def test_36_to_1_speed_and_position_scaling(self) -> None:
        driver = FakeDriver(multi_turn_deg=1000.0)
        joint = initialized_joint(driver, make_config())
        driver.single_samples = [single_for_output(350.0)]
        joint.command_position(math.radians(1), math.radians(2))
        target, speed = driver.commands[0]
        self.assertAlmostEqual(target, 1036.0)
        self.assertAlmostEqual(speed, 72.0)


class JointCliSafetyTests(unittest.TestCase):
    def common_args(self) -> list[str]:
        return [
            "--motor-id",
            "1",
            "--target-rad",
            "0.1",
            "--velocity-rad-s",
            "0.05",
            "--encoder-zero-output-deg",
            "350",
            "--direction-sign",
            "1",
            "--min-position-rad",
            str(math.radians(-20)),
            "--max-position-rad",
            str(math.radians(40)),
        ]

    def test_explicit_dry_run_never_opens_bus_even_if_motion_flag_is_present(self) -> None:
        args = self.common_args() + [
            "--dry-run",
            "--enable-motion",
            "--current-circle-angle-raw",
            "1260000",
            "--current-multi-turn-deg",
            "100",
        ]
        with (
            patch.object(joint_cli, "CanMotorBus") as bus_class,
            redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(joint_cli.main(args), 0)
        bus_class.assert_not_called()

    def test_named_config_dry_run_uses_configured_motor(self) -> None:
        args = [
            "--joint",
            "elbow",
            "--target-rad",
            "0",
            "--velocity-rad-s",
            str(math.radians(1)),
            "--dry-run",
            "--current-circle-angle-raw",
            "324000",
            "--current-multi-turn-deg",
            "0",
        ]
        output = io.StringIO()
        with (
            patch.object(joint_cli, "CanMotorBus") as bus_class,
            redirect_stdout(output),
        ):
            self.assertEqual(joint_cli.main(args), 0)
        bus_class.assert_not_called()
        self.assertIn("CAN motor ID                 : 2", output.getvalue())

    def test_without_enable_motion_live_mode_never_calls_command_position(self) -> None:
        fake_bus = MagicMock()
        fake_bus.__enter__.return_value = object()
        fake_joint = MagicMock()
        fake_joint.get_state.return_value = JointState(
            timestamp_monotonic=1.0,
            circle_angle_raw=1_260_000,
            motor_cycle_deg=12_600.0,
            output_abs_deg=350.0,
            position_rad=0.0,
            motor_multi_turn_deg=100.0,
            motor_speed_deg_s=0.0,
            velocity_rad_s=0.0,
            temperature_c=30,
            motor_state=0,
            error_state=0,
            position_valid=True,
            moving=False,
        )
        with (
            patch.object(joint_cli, "CanMotorBus", return_value=fake_bus),
            patch.object(joint_cli, "MG4010Driver"),
            patch.object(joint_cli, "CanRotaryJoint", return_value=fake_joint),
            redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(joint_cli.main(self.common_args()), 0)
        fake_joint.command_position.assert_not_called()

    def test_motion_callback_prints_final_actual_a4_frame(self) -> None:
        message = can.Message(
            arbitration_id=0x141,
            data=bytes.fromhex("A4 00 48 00 1B 82 02 00"),
            is_extended_id=False,
        )
        output = io.StringIO()
        with redirect_stdout(output):
            joint_cli._motion_frame_printer(False)("TX", message)
        self.assertIn(
            "FINAL-MOTION-TX 0x141 [8] A4 00 48 00 1B 82 02 00",
            output.getvalue(),
        )


if __name__ == "__main__":
    unittest.main()
