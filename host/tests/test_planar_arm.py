"""逆运动学与肩肘关节调用层的离线测试。"""

from __future__ import annotations

import math
import unittest
from unittest.mock import MagicMock

from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from kinematics import Planar2RKinematics
from robot import (
    NoJointLimitSolutionError,
    Planar2RArmController,
    PlanarArmCommandError,
    joint_limited_solutions,
    select_joint_target,
)


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


if __name__ == "__main__":
    unittest.main()
