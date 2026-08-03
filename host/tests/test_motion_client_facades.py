"""Pure-mock tests for the two thin motion client façades."""

from __future__ import annotations

import unittest
from unittest.mock import Mock, sentinel

from bootstrap import create_upper_motion_runtime
from motion.client_facades import FrontendMotionFacade, KinematicsMotionFacade
from motion.client_interfaces import (
    FrontendMotionInterface,
    KinematicsMotionInterface,
)
from motion.unified_protocol import AxisName


class FrontendMotionFacadeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = Mock()
        self.facade = FrontendMotionFacade(self.controller)

    def test_construction_has_no_controller_calls(self) -> None:
        self.controller.assert_not_called()
        self.assertEqual(self.controller.method_calls, [])

    def test_query_methods_forward_and_preserve_results(self) -> None:
        self.controller.list_axes.return_value = sentinel.axes
        self.controller.describe_axis.return_value = sentinel.descriptor
        self.controller.get_state.return_value = sentinel.state
        self.controller.get_axis_states.return_value = sentinel.states

        self.assertIs(self.facade.list_axes(), sentinel.axes)
        self.assertIs(
            self.facade.describe_axis(AxisName.SHOULDER), sentinel.descriptor
        )
        self.assertIs(self.facade.get_state(AxisName.ELBOW), sentinel.state)
        self.assertIs(self.facade.get_axis_states(), sentinel.states)
        self.controller.get_axis_states.assert_called_once_with(None)

    def test_submission_and_result_methods_forward_unchanged(self) -> None:
        self.controller.submit_absolute.return_value = sentinel.command_handle
        self.controller.submit_positions.return_value = sentinel.group_handle
        self.controller.get_command_result.return_value = sentinel.command_result
        self.controller.get_group_result.return_value = sentinel.group_result

        self.assertIs(
            self.facade.submit_absolute(sentinel.axis_target),
            sentinel.command_handle,
        )
        self.assertIs(
            self.facade.submit_positions(sentinel.multi_axis_target),
            sentinel.group_handle,
        )
        self.assertIs(
            self.facade.get_command_result(sentinel.command_handle),
            sentinel.command_result,
        )
        self.assertIs(
            self.facade.get_group_result(sentinel.group_handle),
            sentinel.group_result,
        )

    def test_stop_and_home_forward_unchanged(self) -> None:
        self.controller.stop.return_value = sentinel.stop_result
        self.controller.home_reference.return_value = sentinel.home_result
        self.assertIs(self.facade.stop(AxisName.SLIDE), sentinel.stop_result)
        self.assertIs(
            self.facade.home_reference(AxisName.Z, timeout_s=8.0),
            sentinel.home_result,
        )
        self.controller.stop.assert_called_once_with(AxisName.SLIDE)
        self.controller.home_reference.assert_called_once_with(
            AxisName.Z,
            timeout_s=8.0,
        )


class KinematicsMotionFacadeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = Mock()
        self.facade = KinematicsMotionFacade(self.controller)

    def test_only_multi_axis_methods_are_exposed(self) -> None:
        self.assertFalse(hasattr(self.facade, "submit_absolute"))
        self.assertFalse(hasattr(self.facade, "stop"))
        self.assertFalse(hasattr(self.facade, "home_reference"))

    def test_all_methods_forward_and_preserve_results(self) -> None:
        axes = (AxisName.SHOULDER, AxisName.ELBOW)
        self.controller.get_axis_states.return_value = sentinel.states
        self.controller.submit_positions.return_value = sentinel.group_handle
        self.controller.get_group_result.return_value = sentinel.group_result
        self.controller.wait_group.return_value = sentinel.wait_result

        self.assertIs(self.facade.get_axis_states(axes), sentinel.states)
        self.assertIs(
            self.facade.submit_positions(sentinel.target), sentinel.group_handle
        )
        self.assertIs(
            self.facade.get_group_result(sentinel.group_handle),
            sentinel.group_result,
        )
        self.assertIs(
            self.facade.wait_group(sentinel.group_handle, timeout_s=10.0),
            sentinel.wait_result,
        )
        self.controller.get_axis_states.assert_called_once_with(axes)
        self.controller.wait_group.assert_called_once_with(
            sentinel.group_handle,
            timeout_s=10.0,
        )
        self.assertFalse(self.controller.submit_absolute.called)


class UpperMotionRuntimeTests(unittest.TestCase):
    def test_runtime_builds_two_views_over_exactly_one_controller(self) -> None:
        controller = Mock()
        runtime = create_upper_motion_runtime(controller)
        self.assertIsInstance(runtime.frontend_motion, FrontendMotionInterface)
        self.assertIsInstance(runtime.kinematics_motion, KinematicsMotionInterface)
        self.assertIs(runtime.frontend_motion._controller, controller)
        self.assertIs(runtime.kinematics_motion._controller, controller)
        self.assertIs(runtime._controller, controller)
        self.assertEqual(controller.method_calls, [])

    def test_runtime_client_members_are_read_only(self) -> None:
        runtime = create_upper_motion_runtime(Mock())
        with self.assertRaises(AttributeError):
            runtime.frontend_motion = Mock()  # type: ignore[misc]
        with self.assertRaises(AttributeError):
            runtime.kinematics_motion = Mock()  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
