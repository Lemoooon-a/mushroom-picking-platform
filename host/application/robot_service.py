"""机器人进程级唯一入口：生命周期、状态、视觉与抓取工作流编排。"""

from __future__ import annotations

from dataclasses import dataclass

from application.controller import MushroomRobotController
from application.execution_record import ExecutionRecorder, NullExecutionRecorder
from application.grasp_profile import GraspProfile
from application.motion_target import BaseToolTarget
from application.pick_planner import PickPlan
from application.pick_workflow import PickOutcome, PickResult, VisionPickWorkflow
from application.runtime_state import RobotServiceMode, RobotServiceState
from calibration.hand_eye import HandEyeCalibrationStatus
from vision.observation import VisionTargetObservation


class RobotServiceError(RuntimeError):
    pass


class RobotServiceStateError(RobotServiceError):
    pass


class RobotServiceCapabilityError(RobotServiceError):
    pass


@dataclass(frozen=True)
class RobotServiceCapabilities:
    base_frame_motion: bool
    tray_workspace_gate: bool
    offset_planning: bool
    robot_motion_envelope: bool
    joint_holding: bool
    suction_command: bool
    vision_gateway: str
    vision_target_observation: bool
    hand_eye_calibration: HandEyeCalibrationStatus
    vision_target_resolution: bool
    pick_planning: bool
    pick_execution: bool
    physical_pick_verification: bool


@dataclass(frozen=True)
class RobotServiceStatus:
    state: RobotServiceState
    mode: RobotServiceMode
    capabilities: RobotServiceCapabilities
    backend_status: object | None
    fault: str | None


@dataclass(frozen=True)
class MotionResult:
    executed: bool
    plan: object
    message: str


class MushroomRobotService:
    def __init__(
        self,
        *,
        controller: MushroomRobotController,
        workflow: VisionPickWorkflow | None,
        mode: RobotServiceMode,
        grasp_profile: GraspProfile | None = None,
        runtime: object | None = None,
        recorder: ExecutionRecorder | None = None,
        activate_controller_on_startup: bool | None = None,
        vision_gateway_description: str = "unavailable",
    ) -> None:
        if not isinstance(controller, MushroomRobotController):
            raise TypeError("controller must be a MushroomRobotController")
        if workflow is not None and not isinstance(workflow, VisionPickWorkflow):
            raise TypeError("workflow must be VisionPickWorkflow or None")
        if not isinstance(mode, RobotServiceMode):
            raise TypeError("mode must be a RobotServiceMode")
        if grasp_profile is not None and not isinstance(grasp_profile, GraspProfile):
            raise TypeError("grasp_profile must be a GraspProfile or None")
        self.runtime = runtime
        self.controller = controller
        self.workflow = workflow
        self.mode = mode
        self.grasp_profile = grasp_profile
        self.recorder = recorder or NullExecutionRecorder()
        self.activate_controller_on_startup = (
            mode is not RobotServiceMode.READ_ONLY
            if activate_controller_on_startup is None
            else bool(activate_controller_on_startup)
        )
        self.vision_gateway_description = vision_gateway_description
        self.state = RobotServiceState.CREATED
        self.fault: str | None = None
        self._started_controller = False

    @property
    def capabilities(self) -> RobotServiceCapabilities:
        base = self.controller.capabilities
        vision_observation = self.workflow is not None
        pick_planning = bool(base.vision_target_resolution and self.grasp_profile is not None and vision_observation)
        return RobotServiceCapabilities(
            base_frame_motion=base.base_frame_motion,
            tray_workspace_gate=True,
            offset_planning=True,
            robot_motion_envelope=True,
            joint_holding=base.rotary_joint_enable_control,
            suction_command=base.suction_control,
            vision_gateway=self.vision_gateway_description,
            vision_target_observation=vision_observation,
            hand_eye_calibration=base.hand_eye_calibration,
            vision_target_resolution=base.vision_target_resolution,
            pick_planning=pick_planning,
            pick_execution=pick_planning and self.mode is RobotServiceMode.EXECUTE,
            physical_pick_verification=False,
        )

    def startup(self) -> None:
        if self.state not in (RobotServiceState.CREATED, RobotServiceState.SHUTDOWN):
            raise RobotServiceStateError(f"startup requires CREATED/SHUTDOWN, got {self.state.value}")
        self.state = RobotServiceState.STARTING
        try:
            if self.activate_controller_on_startup:
                self.controller.startup()
                self._started_controller = True
            self.fault = None
            self.state = RobotServiceState.READY
            self._record("startup", final_status="ready")
        except Exception as exc:
            self._fault("startup", exc)
            raise

    def shutdown(self) -> None:
        try:
            if self._started_controller:
                self.controller.shutdown()
        finally:
            self._started_controller = False
            self.state = RobotServiceState.SHUTDOWN
            self._record("shutdown", final_status="shutdown")

    def status(self) -> RobotServiceStatus:
        backend_status = None
        if self._started_controller and self.state not in (RobotServiceState.SHUTDOWN, RobotServiceState.CREATED):
            try:
                backend_status = self.controller.get_status()
            except Exception as exc:
                backend_status = f"unavailable: {exc}"
        return RobotServiceStatus(self.state, self.mode, self.capabilities, backend_status, self.fault)

    def plan_base_target(self, target: BaseToolTarget) -> object:
        self._require_ready("plan")
        self._require_not_read_only("plan")
        self.state = RobotServiceState.PLANNING
        try:
            plan = self.controller.plan_base_target(target)
            self.state = RobotServiceState.READY
            self._record("plan", input_target=target, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("plan", input_target=target, final_status="rejected", error=str(exc))
            raise

    def move_base_target(self, target: BaseToolTarget) -> MotionResult:
        plan = self.plan_base_target(target)
        if self.mode is not RobotServiceMode.EXECUTE:
            return MotionResult(False, plan, "Dry-run plan complete; no motion command was submitted.")
        self.state = RobotServiceState.EXECUTING
        try:
            self.controller.execute_base_plan(plan)
            self.state = RobotServiceState.READY
            result = MotionResult(True, plan, "Base motion completed.")
            self._record("move", input_target=target, selected_plan=plan, final_status="ready")
            return result
        except Exception as exc:
            self._best_effort_stop()
            self._fault("move", exc)
            raise

    def request_observation(self) -> VisionTargetObservation:
        self._require_ready("observe")
        self._require_not_read_only("observe")
        workflow = self._require_workflow()
        self.state = RobotServiceState.OBSERVING
        try:
            observation = workflow.request_observation()
            self.state = RobotServiceState.READY
            self._record("observe", request_id=observation.request_id, final_status="ready")
            return observation
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("observe", final_status="rejected", error=str(exc))
            raise

    def plan_observation(self, observation: VisionTargetObservation, grasp_profile: GraspProfile | None = None) -> PickPlan:
        self._require_ready("plan-observation")
        self._require_not_read_only("plan-observation")
        if not self.controller.capabilities.vision_target_resolution:
            raise RobotServiceCapabilityError(
                "Hand-eye calibration is missing or not validated."
            )
        profile = self._require_profile(grasp_profile)
        workflow = self._require_workflow()
        self.state = RobotServiceState.PLANNING
        try:
            plan = workflow.plan_observation(observation, profile)
            self.state = RobotServiceState.READY
            self._record("plan-observation", request_id=observation.request_id, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("plan-observation", request_id=observation.request_id, final_status="rejected", error=str(exc))
            raise

    def execute_pick_plan(self, plan: PickPlan) -> PickResult:
        self._require_ready("pick")
        workflow = self._require_workflow()
        execute = self.mode is RobotServiceMode.EXECUTE
        self.state = RobotServiceState.EXECUTING if execute else RobotServiceState.PLANNING
        result = workflow.execute_pick_plan(plan, execute=execute)
        if result.outcome is PickOutcome.FAILED:
            self.fault = result.message
            self.state = RobotServiceState.FAULT
        else:
            self.state = RobotServiceState.READY
        self._record("pick", request_id=plan.observation.request_id, selected_plan=plan, final_status=self.state.value, stage_result=result)
        return result

    def pick(self, grasp_profile: GraspProfile | None = None) -> PickResult:
        observation = self.request_observation()
        plan = self.plan_observation(observation, grasp_profile)
        return self.execute_pick_plan(plan)

    def return_to_startup(self) -> object:
        self._require_ready("return")
        self._require_execute("return")
        self.state = RobotServiceState.EXECUTING
        try:
            result = self.controller.return_to_startup()
            self.state = RobotServiceState.READY
            return result
        except Exception as exc:
            self._best_effort_stop()
            self._fault("return", exc)
            raise

    def stop(self) -> None:
        if self.mode is RobotServiceMode.EXECUTE and self._started_controller:
            try:
                self.controller.stop()
            except Exception as exc:
                self._fault("stop", exc)
                raise
        if self.state is not RobotServiceState.FAULT:
            self.state = RobotServiceState.READY
        self._record("stop", final_status=self.state.value)

    def enable_joints(self) -> object:
        self._require_execute("joints enable")
        result = self.controller.enable_joints()
        self.state = RobotServiceState.READY
        return result

    def disable_joints(self) -> object:
        self._require_execute("joints disable")
        result = self.controller.disable_joints()
        self.state = RobotServiceState.DISABLED
        return result

    def suction(self, action: str) -> object:
        self._require_ready(f"suction {action}")
        self._require_execute(f"suction {action}")
        methods = {
            "grip": self.controller.suction_grip,
            "release": self.controller.suction_release,
            "idle": self.controller.suction_idle,
        }
        if action not in methods:
            raise ValueError("suction action must be grip, release, or idle")
        try:
            return methods[action]()
        except Exception as exc:
            self._best_effort_stop()
            self._fault(f"suction {action}", exc)
            raise

    def _require_ready(self, operation: str) -> None:
        if self.state is not RobotServiceState.READY:
            raise RobotServiceStateError(f"{operation} requires READY, got {self.state.value}")

    def _require_not_read_only(self, operation: str) -> None:
        if self.mode is RobotServiceMode.READ_ONLY:
            raise RobotServiceCapabilityError(f"{operation} is unavailable in read-only mode")

    def _require_execute(self, operation: str) -> None:
        self._require_ready(operation)
        if self.mode is not RobotServiceMode.EXECUTE:
            raise RobotServiceCapabilityError(f"{operation} requires execute mode and explicit motion authorization")

    def _require_workflow(self) -> VisionPickWorkflow:
        if self.workflow is None:
            raise RobotServiceCapabilityError("Vision gateway/workflow is unavailable.")
        return self.workflow

    def _require_profile(self, provided: GraspProfile | None) -> GraspProfile:
        profile = provided or self.grasp_profile
        if profile is None:
            raise RobotServiceCapabilityError("Grasp profile is missing or not validated.")
        return profile

    def _best_effort_stop(self) -> None:
        try:
            self.controller.stop()
        except Exception:
            pass

    def _fault(self, operation: str, exc: Exception) -> None:
        self.fault = f"{operation}: {exc}"
        self.state = RobotServiceState.FAULT
        self._record(operation, final_status="fault", error=str(exc))

    def _record(self, operation: str, **fields: object) -> None:
        self.recorder.record(operation, application_state=self.state, **fields)


__all__ = [
    "MotionResult", "MushroomRobotService", "RobotServiceCapabilities", "RobotServiceCapabilityError",
    "RobotServiceError", "RobotServiceStateError", "RobotServiceStatus",
]
