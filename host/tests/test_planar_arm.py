"""逆运动学与肩肘关节调用层的离线测试。"""

from __future__ import annotations

from contextlib import redirect_stdout
import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from kinematics import Planar2RKinematics
from robot import (
    NoJointLimitSolutionError,
    Planar2RArmController,
    PlanarArmCommandError,
    joint_limited_solutions,
    select_joint_target,
)
from scripts import test_planar_2r_motion as arm_cli


class PlanarArmPlanningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.kinematics = Planar2RKinematics(1.0, 1.0)

    def test_joint_limited_solutions_keep_both_legal_branches(self) -> None:
        point = self.kinematics.forward(
            math.radians(20.0),
            math.radians(20.0),
        )
        solutions = joint_limited_solutions(
            self.kinematics,
            SHOULDER_JOINT_CONFIG,
            ELBOW_JOINT_CONFIG,
            point.x,
            point.y,
        )
        self.assertEqual(len(solutions), 2)

    def test_selects_requested_positive_and_negative_elbow_branches(self) -> None:
        point = self.kinematics.forward(
            math.radians(20.0),
            math.radians(20.0),
        )
        positive = select_joint_target(
            self.kinematics,
            SHOULDER_JOINT_CONFIG,
            ELBOW_JOINT_CONFIG,
            point.x,
            point.y,
            elbow_branch="positive",
        )
        negative = select_joint_target(
            self.kinematics,
            SHOULDER_JOINT_CONFIG,
            ELBOW_JOINT_CONFIG,
            point.x,
            point.y,
            elbow_branch="negative",
        )
        self.assertGreater(positive.angles.elbow_rad, 0.0)
        self.assertLess(negative.angles.elbow_rad, 0.0)

    def test_rejects_target_when_all_solutions_violate_joint_limits(self) -> None:
        with self.assertRaises(NoJointLimitSolutionError):
            select_joint_target(
                self.kinematics,
                SHOULDER_JOINT_CONFIG,
                ELBOW_JOINT_CONFIG,
                -2.0,
                0.0,
            )


class PlanarArmCommandTests(unittest.TestCase):
    def make_joint(self, config: object) -> MagicMock:
        joint = MagicMock()
        joint.config = config
        return joint

    def test_prevalidates_both_joints_before_first_submission(self) -> None:
        shoulder = self.make_joint(SHOULDER_JOINT_CONFIG)
        elbow = self.make_joint(ELBOW_JOINT_CONFIG)
        elbow.validate_position_command.side_effect = ValueError("bad elbow")
        controller = Planar2RArmController(
            Planar2RKinematics(1.0, 1.0),
            shoulder,
            elbow,
        )
        target = controller.plan_target(1.0, 1.0)
        with self.assertRaisesRegex(ValueError, "bad elbow"):
            controller.command_target(
                target,
                shoulder_velocity_rad_s=0.1,
                elbow_velocity_rad_s=0.1,
            )
        shoulder.command_position.assert_not_called()
        elbow.command_position.assert_not_called()

    def test_partial_submission_failure_attempts_to_stop_both_joints(self) -> None:
        shoulder = self.make_joint(SHOULDER_JOINT_CONFIG)
        elbow = self.make_joint(ELBOW_JOINT_CONFIG)
        elbow.command_position.side_effect = RuntimeError("elbow failed")
        controller = Planar2RArmController(
            Planar2RKinematics(1.0, 1.0),
            shoulder,
            elbow,
        )
        target = controller.plan_target(1.0, 1.0)
        with self.assertRaises(PlanarArmCommandError):
            controller.command_target(
                target,
                shoulder_velocity_rad_s=0.1,
                elbow_velocity_rad_s=0.1,
            )
        shoulder.stop.assert_called_once_with()
        elbow.stop.assert_called_once_with()


class PlanarArmCliTests(unittest.TestCase):
    def test_preview_is_fully_offline(self) -> None:
        args = [
            "--link1-length",
            "1",
            "--link2-length",
            "1",
            "--x",
            "1",
            "--y",
            "1",
            "--velocity-rad-s",
            "0.1",
        ]
        output = io.StringIO()
        with (
            patch.object(arm_cli, "CanMotorBus") as bus_class,
            redirect_stdout(output),
        ):
            self.assertEqual(arm_cli.main(args), 0)
        bus_class.assert_not_called()
        self.assertIn("OFFLINE PREVIEW", output.getvalue())

    def test_live_mode_initializes_and_commands_both_joints(self) -> None:
        args = [
            "--link1-length",
            "1",
            "--link2-length",
            "1",
            "--x",
            "1",
            "--y",
            "1",
            "--velocity-rad-s",
            "0.1",
            "--enable-motion",
        ]
        fake_bus = MagicMock()
        fake_bus.__enter__.return_value = object()
        shoulder = MagicMock()
        shoulder.config = SHOULDER_JOINT_CONFIG
        shoulder.initialize.return_value = SimpleNamespace(position_rad=0.0)
        elbow = MagicMock()
        elbow.config = ELBOW_JOINT_CONFIG
        elbow.initialize.return_value = SimpleNamespace(position_rad=0.0)
        with (
            patch.object(arm_cli, "CanMotorBus", return_value=fake_bus),
            patch.object(arm_cli, "MG4010Driver"),
            patch.object(
                arm_cli,
                "CanRotaryJoint",
                side_effect=(shoulder, elbow),
            ),
            redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(arm_cli.main(args), 0)
        shoulder.initialize.assert_called_once_with()
        elbow.initialize.assert_called_once_with()
        shoulder.command_position.assert_called_once()
        elbow.command_position.assert_called_once()

    def test_motion_printer_always_shows_final_a4(self) -> None:
        message = arm_cli.can.Message(
            arbitration_id=0x141,
            data=bytes.fromhex("A4 00 48 00 1B 82 02 00"),
            is_extended_id=False,
        )
        output = io.StringIO()
        with redirect_stdout(output):
            arm_cli._motion_frame_printer(False)("TX", message)
        self.assertIn("FINAL-CONTROL-TX 0x141", output.getvalue())


if __name__ == "__main__":
    unittest.main()
