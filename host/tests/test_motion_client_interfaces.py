"""Contract tests for the stable frontend and kinematics protocols."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest
from typing import get_type_hints

from motion.client_facades import FrontendMotionFacade, KinematicsMotionFacade
from motion.client_interfaces import (
    FrontendMotionInterface,
    KinematicsMotionInterface,
)
from motion.unified_protocol import (
    AxisState,
    AxisTarget,
    MultiAxisCommandResult,
    MultiAxisTarget,
)


class MotionClientInterfaceTests(unittest.TestCase):
    def test_facades_satisfy_runtime_checkable_protocols(self) -> None:
        controller = object()
        self.assertIsInstance(
            FrontendMotionFacade(controller),  # type: ignore[arg-type]
            FrontendMotionInterface,
        )
        self.assertIsInstance(
            KinematicsMotionFacade(controller),  # type: ignore[arg-type]
            KinematicsMotionInterface,
        )

    def test_frontend_protocol_has_exact_public_motion_members(self) -> None:
        members = {
            name
            for name, value in FrontendMotionInterface.__dict__.items()
            if callable(value) and not name.startswith("_")
        }
        self.assertEqual(
            members,
            {
                "list_axes",
                "describe_axis",
                "get_state",
                "get_axis_states",
                "submit_absolute",
                "submit_positions",
                "get_command_result",
                "get_group_result",
                "stop",
                "home_reference",
                "suction_grip",
                "suction_release",
                "suction_idle",
                "get_suction_status",
                "enable_rotary_joints",
                "disable_rotary_joints",
                "rotary_joints_enabled",
                "get_rotary_joint_enable_status",
            },
        )

    def test_kinematics_protocol_remains_narrow(self) -> None:
        members = {
            name
            for name, value in KinematicsMotionInterface.__dict__.items()
            if callable(value) and not name.startswith("_")
        }
        self.assertEqual(
            members,
            {
                "get_axis_states",
                "submit_positions",
                "get_group_result",
                "wait_group",
            },
        )
        self.assertFalse({"home_reference", "stop", "submit_absolute"} & members)

    def test_protocols_reuse_unified_dtos(self) -> None:
        frontend_submit = get_type_hints(
            FrontendMotionInterface.submit_absolute
        )
        kinematics_submit = get_type_hints(
            KinematicsMotionInterface.submit_positions
        )
        kinematics_states = get_type_hints(
            KinematicsMotionInterface.get_axis_states
        )
        kinematics_wait = get_type_hints(KinematicsMotionInterface.wait_group)
        self.assertIs(frontend_submit["target"], AxisTarget)
        self.assertIs(kinematics_submit["target"], MultiAxisTarget)
        self.assertEqual(kinematics_states["return"], tuple[AxisState, ...])
        self.assertIs(kinematics_wait["return"], MultiAxisCommandResult)

    def test_interface_module_has_no_transport_or_driver_imports(self) -> None:
        source_path = Path(__file__).parents[1] / "motion" / "client_interfaces.py"
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imported_modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module)
        self.assertFalse(
            any(
                name == "drivers"
                or name.startswith("drivers.")
                or "transport" in name
                for name in imported_modules
            )
        )

    def test_handoff_documents_contain_required_boundary_terms(self) -> None:
        docs_dir = Path(__file__).parents[2] / "docs" / "handoffs"
        for filename in (
            "FRONTEND_MOTION_INTERFACE_HANDOFF.md",
            "KINEMATICS_MOTION_INTERFACE_HANDOFF.md",
        ):
            with self.subTest(filename=filename):
                text = (docs_dir / filename).read_text(encoding="utf-8").lower()
                for term in ("mm", "deg", "accepted", "arrived", "startup"):
                    self.assertIn(term, text)


if __name__ == "__main__":
    unittest.main()
