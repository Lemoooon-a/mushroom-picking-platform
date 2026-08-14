"""同步视觉观察与抓取阶段执行；不实现 FK、IK 或执行器协议。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import itertools
import time
from collections.abc import Callable

from application.controller import MushroomRobotController
from application.grasp_profile import GraspProfile
from application.pick_planner import PickPlan, PickPlanner
from kinematics.frame_chain import RobotAxisState
from vision.gateway import VisionGateway
from vision.observation import (
    CaptureMotionState, VisionTargetObservation, create_capture_snapshot,
    require_snapshot_unchanged,
)
from vision.protocol import CaptureRequest, NoTarget, TargetDetection, VisionError


class PickOutcome(str, Enum):
    PLANNED = "planned"
    MOTION_COMPLETED = "motion_completed"
    SUCTION_COMMAND_ACKNOWLEDGED = "suction_command_acknowledged"
    PHYSICAL_PICK_UNVERIFIED = "physical_pick_unverified"
    FAILED = "failed"


@dataclass(frozen=True)
class PickResult:
    outcome: PickOutcome
    observation: VisionTargetObservation | None
    plan: PickPlan | None
    message: str


class VisionWorkflowError(RuntimeError):
    pass


class NoVisionTarget(VisionWorkflowError):
    pass


CaptureStateReader = Callable[[], tuple[RobotAxisState, CaptureMotionState]]


class VisionPickWorkflow:
    def __init__(
        self,
        *,
        controller: MushroomRobotController,
        gateway: VisionGateway,
        planner: PickPlanner,
        capture_state_reader: CaptureStateReader,
        joints_holding: Callable[[], bool],
        camera_frame: str,
        timeout_s: float = 2.0,
        clock=time.time,
    ) -> None:
        if not isinstance(controller, MushroomRobotController):
            raise TypeError("controller must be a MushroomRobotController")
        if not isinstance(gateway, VisionGateway):
            raise TypeError("gateway must implement VisionGateway")
        if not isinstance(planner, PickPlanner):
            raise TypeError("planner must be a PickPlanner")
        if not isinstance(camera_frame, str) or not camera_frame.strip():
            raise ValueError("camera_frame must be a non-empty string")
        self.controller = controller
        self.gateway = gateway
        self.planner = planner
        self.capture_state_reader = capture_state_reader
        self.joints_holding = joints_holding
        self.camera_frame = camera_frame
        self.timeout_s = float(timeout_s)
        self.clock = clock
        self._ids = itertools.count(1)

    def request_observation(self) -> VisionTargetObservation:
        if not self.joints_holding():
            raise VisionWorkflowError("rotary joints must be holding before capture")
        before_state, before_motion = self.capture_state_reader()
        captured_at = float(self.clock())
        request_id = f"capture-{next(self._ids):06d}"
        resolver = self.controller.target_resolver
        if resolver is None:
            # 观察本身允许缺手眼；快照中的 Base pose 此时仍由机器人 FK 提供。
            raise VisionWorkflowError("capture pose provider is unavailable")
        snapshot = create_capture_snapshot(
            request_id=request_id,
            axis_state=before_state,
            base_T_tool=resolver.pose_provider.forward_kinematics_base(before_state),
            captured_at=captured_at,
            motion_state=before_motion,
        )
        result = self.gateway.request_target(
            CaptureRequest(request_id, self.camera_frame, captured_at), self.timeout_s
        )
        after_state, after_motion = self.capture_state_reader()
        require_snapshot_unchanged(snapshot, axis_state=after_state, motion_state=after_motion)
        if isinstance(result, NoTarget):
            raise NoVisionTarget(result.reason)
        if isinstance(result, VisionError):
            raise VisionWorkflowError(f"vision error {result.code}: {result.message}")
        if not isinstance(result, TargetDetection):
            raise VisionWorkflowError("unsupported vision result")
        if result.frame_id != self.camera_frame:
            raise VisionWorkflowError(
                f"observation frame_id={result.frame_id!r} does not match {self.camera_frame!r}"
            )
        return VisionTargetObservation(
            request_id=result.request_id,
            frame_id=result.frame_id,
            timestamp=result.timestamp,
            position_mm=result.position_mm,
            orientation=result.orientation,
            confidence=result.confidence,
            target_id=result.target_id,
            capture_axis_state=snapshot.axis_state,
            capture_motion_state=CaptureMotionState.STATIONARY,
        )

    def plan_observation(self, observation: VisionTargetObservation, grasp_profile: GraspProfile) -> PickPlan:
        return self.planner.plan(observation, grasp_profile)

    def execute_pick_plan(
        self,
        plan: PickPlan,
        *,
        execute: bool,
        continue_check: Callable[[], bool] | None = None,
    ) -> PickResult:
        if not isinstance(plan, PickPlan):
            raise TypeError("plan must be a PickPlan")
        if not execute:
            return PickResult(PickOutcome.PLANNED, plan.observation, plan, "Pick plan validated; no motion command was submitted.")
        try:
            _require_execution_continues(continue_check, "overhead motion")
            self.controller.execute_base_plan(plan.overhead_motion)
            _require_execution_continues(continue_check, "contact motion")
            self.controller.execute_base_plan(plan.contact_motion)
            _require_execution_continues(continue_check, "suction")
            self.controller.suction_grip()
            time.sleep(plan.suction_settle_time_s)
            _require_execution_continues(continue_check, "lift motion")
            self.controller.execute_base_plan(plan.lift_motion)
        except Exception as exc:
            try:
                self.controller.stop()
            except Exception:
                pass
            return PickResult(PickOutcome.FAILED, plan.observation, plan, f"pick stage failed; best-effort stop requested: {exc}")
        return PickResult(
            PickOutcome.PHYSICAL_PICK_UNVERIFIED,
            plan.observation,
            plan,
            "Motion completed and suction command acknowledged; physical pick is unverified because no vacuum feedback is available.",
        )

    def run(self, grasp_profile: GraspProfile, *, execute: bool = False) -> PickResult:
        observation = self.request_observation()
        plan = self.plan_observation(observation, grasp_profile)
        return self.execute_pick_plan(plan, execute=execute)


def _require_execution_continues(
    continue_check: Callable[[], bool] | None,
    next_stage: str,
) -> None:
    if continue_check is not None and not continue_check():
        raise VisionWorkflowError(f"pick execution cancelled before {next_stage}")


__all__ = [
    "NoVisionTarget", "PickOutcome", "PickResult", "VisionPickWorkflow", "VisionWorkflowError",
]
