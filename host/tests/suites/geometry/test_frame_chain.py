from __future__ import annotations

import unittest

import numpy as np

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import (
    MissingToolCameraTransformError,
    RobotAxisState,
    RobotFrameChain,
)


class FakeKinematics:
    def __init__(self, transform: RigidTransform) -> None:
        self.transform = transform
        self.calls: list[RobotAxisState] = []

    def forward_kinematics(self, axis_state: RobotAxisState) -> RigidTransform:
        self.calls.append(axis_state)
        return self.transform


class FrameChainTests(unittest.TestCase):
    def setUp(self) -> None:
        self.axis_state = RobotAxisState(1, 2, 3, 4, 5)
        self.base_T_slide_zero = RigidTransform.from_xyz_yaw_deg(
            x_mm=100, y_mm=200, z_mm=300, yaw_deg=90
        )
        self.slide_zero_T_tool = RigidTransform.from_xyz_yaw_deg(
            x_mm=10, y_mm=0, z_mm=20, yaw_deg=20
        )
        self.tool_T_camera = RigidTransform.from_xyz_rpy_deg(
            x_mm=0,
            y_mm=5,
            z_mm=10,
            roll_deg=10,
            pitch_deg=20,
            yaw_deg=30,
        )
        self.kinematics = FakeKinematics(self.slide_zero_T_tool)
        self.chain = RobotFrameChain(
            base_T_slide_zero=self.base_T_slide_zero,
            tool_T_camera=self.tool_T_camera,
            slide_zero_kinematics=self.kinematics,
        )

    def test_base_tool_composition(self) -> None:
        np.testing.assert_allclose(
            self.chain.base_T_tool(self.axis_state).matrix,
            (self.base_T_slide_zero @ self.slide_zero_T_tool).matrix,
        )

    def test_public_forward_kinematics_is_base_rooted(self) -> None:
        np.testing.assert_allclose(
            self.chain.forward_kinematics_base(self.axis_state).matrix,
            self.chain.base_T_tool(self.axis_state).matrix,
        )

    def test_base_camera_composition(self) -> None:
        np.testing.assert_allclose(
            self.chain.base_T_camera(self.axis_state).matrix,
            (
                self.base_T_slide_zero
                @ self.slide_zero_T_tool
                @ self.tool_T_camera
            ).matrix,
        )

    def test_camera_point_to_base(self) -> None:
        point = (1, 2, 3)
        expected = (
            self.base_T_slide_zero
            @ self.slide_zero_T_tool
            @ self.tool_T_camera
        ).transform_point(point)
        np.testing.assert_allclose(
            self.chain.transform_camera_point_to_base(point, self.axis_state),
            expected,
        )

    def test_base_target_to_slide_zero(self) -> None:
        base_T_target = RigidTransform.from_xyz_yaw_deg(
            x_mm=2, y_mm=4, z_mm=6, yaw_deg=8
        )
        result = self.chain.transform_base_target_to_slide_zero(base_T_target)
        np.testing.assert_allclose(
            (self.base_T_slide_zero @ result).matrix,
            base_T_target.matrix,
            atol=1e-12,
        )

    def test_missing_tool_camera_is_explicit(self) -> None:
        chain = RobotFrameChain(
            base_T_slide_zero=self.base_T_slide_zero,
            tool_T_camera=None,
            slide_zero_kinematics=self.kinematics,
        )
        with self.assertRaises(MissingToolCameraTransformError):
            chain.base_T_camera(self.axis_state)

    def test_chain_only_uses_provided_axis_state(self) -> None:
        self.chain.base_T_tool(self.axis_state)
        self.assertEqual(self.kinematics.calls, [self.axis_state])

    def test_axis_state_rejects_non_finite_value(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            RobotAxisState(0, 0, 0, 0, float("nan"))


if __name__ == "__main__":
    unittest.main()
