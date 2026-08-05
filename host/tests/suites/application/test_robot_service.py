from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from application.controller import MushroomRobotController
from application.execution_record import JsonLinesExecutionRecorder
from application.motion_target import BaseToolTarget
from application.robot_service import (
    MushroomRobotService, RobotServiceCapabilityError,
)
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.tray_workspace import TrayWorkspace
from config.tray_workspace import TrayWorkspaceConfig


class _Backend:
    def __init__(self) -> None:
        self.calls = []
        self.plan_error = None
        self.execute_error = None

    def startup(self): self.calls.append("startup")
    def require_base_motion_ready(self): self.calls.append("ready")
    def plan_to_base_pose(self, x, y, z, yaw):
        self.calls.append(("plan", x, y, z, yaw))
        if self.plan_error: raise self.plan_error
        return "plan"
    def execute_base_plan(self, plan):
        self.calls.append(("execute", plan))
        if self.execute_error: raise self.execute_error
        return True
    def return_to_startup(self): self.calls.append("return")
    def stop(self): self.calls.append("stop")
    def enable_joints(self): self.calls.append("enable")
    def disable_joints(self): self.calls.append("disable")
    def suction_grip(self): self.calls.append("grip")
    def suction_release(self): self.calls.append("release")
    def suction_idle(self): self.calls.append("idle")
    def get_status(self): self.calls.append("status"); return "ok"
    def shutdown(self): self.calls.append("shutdown")


class RobotServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = _Backend()
        self.controller = MushroomRobotController(
            base_backend=self.backend,
            tray_workspace=TrayWorkspace(TrayWorkspaceConfig(-10, 10, -10, 10, -10, 10)),
        )

    def service(self, mode, **kwargs):
        return MushroomRobotService(controller=self.controller, workflow=None, mode=mode, **kwargs)

    def test_read_only_startup_never_activates_backend(self) -> None:
        service = self.service(RobotServiceMode.READ_ONLY)
        service.startup()
        self.assertIs(service.state, RobotServiceState.READY)
        self.assertEqual(self.backend.calls, [])
        with self.assertRaisesRegex(RobotServiceCapabilityError, "read-only"):
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))

    def test_dry_run_plans_without_submit(self) -> None:
        service = self.service(RobotServiceMode.DRY_RUN)
        service.startup()
        result = service.move_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertFalse(result.executed)
        self.assertNotIn(("execute", "plan"), self.backend.calls)
        self.assertIs(service.state, RobotServiceState.READY)

    def test_execute_motion_failure_stops_and_faults(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.execute_error = RuntimeError("failed")
        with self.assertRaisesRegex(RuntimeError, "failed"):
            service.move_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", self.backend.calls)
        service.stop()
        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_planning_rejection_returns_ready_and_disable_state(self) -> None:
        service = self.service(RobotServiceMode.EXECUTE)
        service.startup()
        self.backend.plan_error = ValueError("no plan")
        with self.assertRaises(ValueError):
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))
        self.assertIs(service.state, RobotServiceState.READY)
        self.backend.plan_error = None
        service.disable_joints()
        self.assertIs(service.state, RobotServiceState.DISABLED)

    def test_missing_hand_eye_message_is_explicit(self) -> None:
        service = self.service(RobotServiceMode.DRY_RUN)
        service.startup()
        with self.assertRaisesRegex(RobotServiceCapabilityError, "Hand-eye calibration is missing or not validated"):
            service.plan_observation(object())

    def test_jsonl_record_is_only_written_to_explicit_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "records.jsonl"
            service = self.service(
                RobotServiceMode.DRY_RUN,
                recorder=JsonLinesExecutionRecorder(path),
            )
            service.startup()
            service.plan_base_target(BaseToolTarget(1, 2, 3, 4))
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([item["operation"] for item in records], ["startup", "plan"])
        self.assertEqual(records[-1]["application_state"], "ready")


if __name__ == "__main__":
    unittest.main()
