from __future__ import annotations

import threading
import unittest
from unittest.mock import Mock

from application.controller import MushroomRobotController
from application.motion_target import BaseToolTarget
from application.robot_service import MushroomRobotService, RobotServiceStateError
from application.pick_planner import PickPlan
from application.pick_workflow import PickOutcome, PickResult, VisionPickWorkflow
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
        self.startup_error: Exception | None = None
        self.enable_error: Exception | None = None
        self.disable_error: Exception | None = None
        self.action_entered = threading.Event()
        self.action_release = threading.Event()
        self.block_action: str | None = None

    def _action(self, name: str) -> None:
        self.calls.append(name)
        if self.block_action == name:
            self.action_entered.set()
            self.action_release.wait(2.0)

    def startup(self):
        self.calls.append("startup")
        if self.startup_error is not None:
            raise self.startup_error
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
    def enable_joints(self):
        self._action("enable")
        if self.enable_error is not None:
            raise self.enable_error
    def disable_joints(self):
        self._action("disable")
        if self.disable_error is not None:
            raise self.disable_error
    def suction_grip(self): self._action("grip")
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
    def make_service(
        self,
        *,
        mode: RobotServiceMode = RobotServiceMode.EXECUTE,
        workflow: VisionPickWorkflow | None = None,
        startup: bool = True,
    ) -> tuple[MushroomRobotService, _Backend, _AxisPort]:
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
            workflow=workflow,
            mode=mode,
            axis_motion=axis_port,
        )
        if startup:
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

    def test_stop_fault_cannot_be_overwritten_by_late_arrival(self) -> None:
        service, backend, axis_port = self.make_service()
        axis_port.block_wait = True
        thread = threading.Thread(
            target=lambda: service.move_axis_relative(AxisName.Z, -10.0)
        )
        thread.start()
        self.assertTrue(axis_port.wait_entered.wait(1.0))
        axis_port.states = _axis_states(valid=False)

        service.stop()
        self.assertIs(service.state, RobotServiceState.FAULT)
        axis_port.wait_release.set()
        thread.join(1.0)

        self.assertFalse(thread.is_alive())
        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", backend.calls)

    def test_shutdown_invalidates_late_arrival_and_is_idempotent(self) -> None:
        service, backend, axis_port = self.make_service()
        axis_port.block_wait = True
        thread = threading.Thread(
            target=lambda: service.move_axis_relative(AxisName.Z, -10.0)
        )
        thread.start()
        self.assertTrue(axis_port.wait_entered.wait(1.0))

        service.shutdown()
        self.assertIs(service.state, RobotServiceState.SHUTDOWN)
        self.assertEqual(backend.calls[-2:], ["stop", "shutdown"])
        axis_port.wait_release.set()
        thread.join(1.0)
        self.assertIs(service.state, RobotServiceState.SHUTDOWN)

        shutdown_count = backend.calls.count("shutdown")
        service.shutdown()
        self.assertEqual(backend.calls.count("shutdown"), shutdown_count)

    def test_status_and_axis_query_remain_available_while_waiting(self) -> None:
        service, _, axis_port = self.make_service()
        axis_port.block_wait = True
        thread = threading.Thread(
            target=lambda: service.move_axis_relative(AxisName.Z, -10.0)
        )
        thread.start()
        self.assertTrue(axis_port.wait_entered.wait(1.0))

        self.assertIs(service.status().state, RobotServiceState.EXECUTING)
        self.assertEqual(len(service.get_axis_states()), len(tuple(AxisName)))
        axis_port.wait_release.set()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())

    def test_stop_without_active_operation_preserves_stable_states(self) -> None:
        created, _, _ = self.make_service(startup=False)
        created.stop()
        self.assertIs(created.state, RobotServiceState.CREATED)

        for state in (
            RobotServiceState.READY,
            RobotServiceState.DISABLED,
            RobotServiceState.FAULT,
        ):
            with self.subTest(state=state.value):
                service, _, _ = self.make_service()
                service.state = state
                service.stop()
                self.assertIs(service.state, state)

        shut, _, _ = self.make_service()
        shut.shutdown()
        shut.stop()
        self.assertIs(shut.state, RobotServiceState.SHUTDOWN)

    def test_disabled_joints_can_be_enabled_and_revalidated(self) -> None:
        service, _, axis_port = self.make_service()
        axis_port.states = _axis_states(rotary_enabled=False)
        service.disable_joints()
        self.assertIs(service.state, RobotServiceState.DISABLED)
        axis_port.states = _axis_states(rotary_enabled=True)

        service.enable_joints()

        self.assertIs(service.state, RobotServiceState.READY)
        self.assertIsNone(service.fault)

    def test_partial_enable_failure_faults(self) -> None:
        service, backend, axis_port = self.make_service()
        axis_port.states = _axis_states(rotary_enabled=False)
        service.disable_joints()
        backend.enable_error = RuntimeError("partial enable")
        mixed = list(_axis_states(rotary_enabled=False))
        shoulder = mixed[list(AxisName).index(AxisName.SHOULDER)]
        mixed[list(AxisName).index(AxisName.SHOULDER)] = AxisState(
            **{**shoulder.__dict__, "enabled": True}
        )
        axis_port.states = tuple(mixed)

        with self.assertRaisesRegex(RuntimeError, "partial enable"):
            service.enable_joints()

        self.assertIs(service.state, RobotServiceState.FAULT)

    def test_suction_disable_and_enable_are_mutually_exclusive_with_move(self) -> None:
        cases = (
            ("grip", lambda service: service.suction("grip")),
            ("disable", lambda service: service.disable_joints()),
            ("enable", lambda service: service.enable_joints()),
        )
        for action, operation in cases:
            with self.subTest(action=action):
                service, backend, axis_port = self.make_service()
                backend.block_action = action
                thread_errors: list[Exception] = []

                def run_action() -> None:
                    try:
                        operation(service)
                    except Exception as exc:
                        thread_errors.append(exc)

                thread = threading.Thread(target=run_action)
                thread.start()
                self.assertTrue(backend.action_entered.wait(1.0))
                with self.assertRaises(RobotServiceStateError):
                    service.move_axis_relative(AxisName.Z, -10.0)
                self.assertNotIn(("axis-submit", AxisName.Z), axis_port.calls)
                backend.action_release.set()
                thread.join(1.0)
                self.assertEqual(thread_errors, [])

    def test_pick_and_base_move_are_mutually_exclusive(self) -> None:
        workflow = Mock(spec=VisionPickWorkflow)
        entered = threading.Event()
        release = threading.Event()
        observation = Mock(request_id="pick-1")
        plan = Mock(spec=PickPlan)
        plan.observation = observation

        def execute(plan_value, *, execute):
            entered.set()
            release.wait(2.0)
            return PickResult(PickOutcome.PLANNED, observation, plan_value, "planned")

        workflow.execute_pick_plan.side_effect = execute
        service, backend, _ = self.make_service(workflow=workflow)
        thread = threading.Thread(target=lambda: service.execute_pick_plan(plan))
        thread.start()
        self.assertTrue(entered.wait(1.0))

        with self.assertRaises(RobotServiceStateError):
            service.move_base_target(BaseToolTarget(1, 2, 3, 0))
        self.assertNotIn("base-submit", backend.calls)
        release.set()
        thread.join(1.0)

    def test_unexpected_pick_exception_does_not_leak_active_state(self) -> None:
        workflow = Mock(spec=VisionPickWorkflow)
        workflow.execute_pick_plan.side_effect = RuntimeError("unexpected pick error")
        plan = Mock(spec=PickPlan)
        plan.observation = Mock(request_id="pick-1")
        service, backend, _ = self.make_service(workflow=workflow)

        with self.assertRaisesRegex(RuntimeError, "unexpected pick error"):
            service.execute_pick_plan(plan)

        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", backend.calls)
        self.assertIsNone(service._active_operation)

    def test_planning_only_pick_exception_returns_ready(self) -> None:
        workflow = Mock(spec=VisionPickWorkflow)
        workflow.execute_pick_plan.side_effect = RuntimeError("planning failed")
        plan = Mock(spec=PickPlan)
        plan.observation = Mock(request_id="pick-1")
        service, backend, _ = self.make_service(
            mode=RobotServiceMode.DRY_RUN,
            workflow=workflow,
        )

        with self.assertRaisesRegex(RuntimeError, "planning failed"):
            service.execute_pick_plan(plan)

        self.assertIs(service.state, RobotServiceState.READY)
        self.assertNotIn("stop", backend.calls)
        self.assertIsNone(service._active_operation)

    def test_startup_failure_stops_closes_and_faults(self) -> None:
        service, backend, _ = self.make_service(startup=False)
        backend.startup_error = RuntimeError("startup failed")

        with self.assertRaisesRegex(RuntimeError, "startup failed"):
            service.startup()

        self.assertIs(service.state, RobotServiceState.FAULT)
        self.assertIn("stop", backend.calls)
        self.assertIn("shutdown", backend.calls)
        self.assertIsNone(service._active_operation)

    def test_shutdown_close_failure_preserves_shutdown_fault(self) -> None:
        service, backend, _ = self.make_service()
        backend.shutdown_error = RuntimeError("close failed")

        with self.assertRaisesRegex(RuntimeError, "close failed"):
            service.shutdown()

        self.assertIs(service.state, RobotServiceState.SHUTDOWN)
        self.assertIn("close failed", service.fault or "")
        close_count = backend.calls.count("shutdown")
        service.shutdown()
        self.assertEqual(backend.calls.count("shutdown"), close_count)


if __name__ == "__main__":
    unittest.main()
