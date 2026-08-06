from __future__ import annotations

import threading
import unittest

from application.controller import MushroomRobotController
from application.motion_target import BaseToolTarget
from application.robot_service import MushroomRobotService, RobotServiceStateError
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.tray_workspace import TrayWorkspace
from config.tray_workspace import TrayWorkspaceConfig
from motion.unified_controller import UnifiedMotionError
from motion.unified_protocol import (
    AxisName,
    AxisState,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
)


def _axis_states(
    *,
    valid: bool = True,
    rotary_enabled: bool = True,
) -> tuple[AxisState, ...]:
    return tuple(
        AxisState(
            axis=axis,
            connected=True,
            enabled=(
                rotary_enabled
                if axis in (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION)
                else True
            ),
            busy=False,
            homed=True if axis in (AxisName.SLIDE, AxisName.Z) else None,
            position_valid=valid,
            current_position=0.0 if valid else None,
            position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
            faulted=False,
            fault_code=None,
            fault_message=None,
        )
        for axis in AxisName
    )


def _result(status: MotionCommandStatus) -> MotionCommandResult:
    error_code = {
        MotionCommandStatus.TIMEOUT: MotionErrorCode.TIMEOUT,
        MotionCommandStatus.FAULT: MotionErrorCode.DEVICE_FAULT,
        MotionCommandStatus.ABORTED: MotionErrorCode.BACKEND_ERROR,
    }.get(status)
    return MotionCommandResult(
        command_id="axis-command",
        axis=AxisName.Z,
        status=status,
        accepted=True,
        completed=True if status is MotionCommandStatus.ARRIVED else False,
        target_position=-10.0,
        final_position=-10.0 if status is MotionCommandStatus.ARRIVED else 0.0,
        position_error=0.0 if status is MotionCommandStatus.ARRIVED else 10.0,
        error_code=error_code,
        message=status.value,
    )


class _Backend:
    def __init__(self) -> None:
        self.calls: list[object] = []
        self.execute_entered = threading.Event()
        self.execute_release = threading.Event()
        self.block_execute = False
        self.shutdown_error: Exception | None = None

    def startup(self): self.calls.append("startup")
    def require_base_motion_ready(self): self.calls.append("ready-check")
    def plan_to_base_pose(self, x, y, z, yaw): self.calls.append("base-plan"); return "plan"
    def execute_base_plan(self, plan):
        self.calls.append("base-submit")
        self.execute_entered.set()
        if self.block_execute:
            self.execute_release.wait(2.0)
        return True
    def return_to_startup(self): self.calls.append("return")
    def stop(self): self.calls.append("stop")
    def enable_joints(self): self.calls.append("enable")
    def disable_joints(self): self.calls.append("disable")
    def suction_grip(self): self.calls.append("grip")
    def suction_release(self): self.calls.append("release")
    def suction_idle(self): self.calls.append("idle")
    def get_status(self): self.calls.append("status"); return "ok"
    def shutdown(self):
        self.calls.append("shutdown")
        if self.shutdown_error is not None:
            raise self.shutdown_error


class _AxisPort:
    def __init__(self) -> None:
        self.calls: list[object] = []
        self.wait_entered = threading.Event()
        self.wait_release = threading.Event()
        self.block_wait = False
        self.wait_error: Exception | None = None
        self.wait_result = _result(MotionCommandStatus.ARRIVED)
        self.states = _axis_states()

    def list_axes(self): return ()
    def get_state(self, axis): return next(item for item in self.states if item.axis is axis)
    def get_axis_states(self, axes=None): self.calls.append("get-states"); return self.states
    def submit_absolute(self, target): self.calls.append(("axis-submit", target.axis)); return MotionCommandHandle("axis-command", target.axis, target.position)
    def submit_relative(self, target): self.calls.append(("axis-submit", target.axis)); return MotionCommandHandle("axis-command", target.axis, -10.0)
    def wait(self, handle, *, timeout_s=None):
        self.calls.append("wait")
        self.wait_entered.set()
        if self.block_wait:
            self.wait_release.wait(2.0)
        if self.wait_error is not None:
            raise self.wait_error
        return self.wait_result
    def stop(self, axis): self.calls.append(("axis-stop", axis)); return _result(MotionCommandStatus.ABORTED)


class RobotServiceStateMachineTests(unittest.TestCase):
    def make_service(self) -> tuple[MushroomRobotService, _Backend, _AxisPort]:
        backend = _Backend()
        axis_port = _AxisPort()
        controller = MushroomRobotController(
            base_backend=backend,
            tray_workspace=TrayWorkspace(
                TrayWorkspaceConfig(-100, 100, -100, 100, -100, 100)
            ),
        )
        service = MushroomRobotService(
            controller=controller,
            workflow=None,
            mode=RobotServiceMode.EXECUTE,
            axis_motion=axis_port,
        )
        service.startup()
        return service, backend, axis_port

    def test_axis_and_base_move_cannot_both_submit(self) -> None:
        service, backend, axis_port = self.make_service()
        backend.block_execute = True
        axis_port.block_wait = True
        start = threading.Barrier(3)
        outcomes: list[object] = []

        def call(operation) -> None:
            start.wait()
            try:
                outcomes.append(operation())
            except Exception as exc:
                outcomes.append(exc)

        threads = (
            threading.Thread(
                target=call,
                args=(lambda: service.move_axis_relative(AxisName.Z, -10.0),),
            ),
            threading.Thread(
                target=call,
                args=(lambda: service.move_base_target(BaseToolTarget(1, 2, 3, 0)),),
            ),
        )
        for thread in threads:
            thread.start()
        start.wait()

        self.assertTrue(
            axis_port.wait_entered.wait(1.0) or backend.execute_entered.wait(1.0)
        )
        for _ in range(100):
            if any(isinstance(item, RobotServiceStateError) for item in outcomes):
                break
            threading.Event().wait(0.005)
        axis_port.wait_release.set()
        backend.execute_release.set()
        for thread in threads:
            thread.join(1.0)

        submissions = int(("axis-submit", AxisName.Z) in axis_port.calls) + int(
            "base-submit" in backend.calls
        )
        self.assertEqual(submissions, 1)
        self.assertEqual(
            sum(isinstance(item, RobotServiceStateError) for item in outcomes), 1
        )
        self.assertIs(service.state, RobotServiceState.READY)

    def test_pre_submission_busy_keeps_ready_without_stop(self) -> None:
        service, _, axis_port = self.make_service()
        axis_port.submit_relative = lambda target: (_ for _ in ()).throw(
            UnifiedMotionError(MotionErrorCode.BUSY, "busy", axis=target.axis)
        )

        with self.assertRaises(UnifiedMotionError):
            service.move_axis_relative(AxisName.Z, -10.0)

        self.assertIs(service.state, RobotServiceState.READY)
        self.assertFalse(any(item == ("axis-stop", AxisName.Z) for item in axis_port.calls))

    def test_post_submission_busy_faults_and_stops(self) -> None:
        service, _, axis_port = self.make_service()
        axis_port.wait_error = UnifiedMotionError(
            MotionErrorCode.BUSY,
            "busy after submit",
            axis=AxisName.Z,
        )

        with self.assertRaises(UnifiedMotionError):
            service.move_axis_relative(AxisName.Z, -10.0)

        self.assertIn(("axis-submit", AxisName.Z), axis_port.calls)
        self.assertIn(("axis-stop", AxisName.Z), axis_port.calls)
        self.assertIs(service.state, RobotServiceState.FAULT)


if __name__ == "__main__":
    unittest.main()
