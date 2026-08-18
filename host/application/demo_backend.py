"""把现有 DemoMotionFlow 薄适配为应用层 Base-frame 后端。"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from application.controller import MushroomRobotController
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import hand_eye_from_frame_document
from config.project.robot_motion_envelope import (
    DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG,
    RobotMotionEnvelopeConfig,
)
from config.robot_runtime import RobotRuntimeConfig
from config.project.workspace_planning import (
    ArmLocalWorkspaceConfig,
    DEFAULT_ARM_LOCAL_WORKSPACE_CONFIG,
)
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.target_resolver import VisionTargetResolver
from vision.observation import CaptureMotionState

if TYPE_CHECKING:
    from scripts.run_motion_demo import DemoMotionFlow


class DemoFlowApplicationBackend:
    """不复制 IK/工作区/阶段执行，仅转发给现有流程。"""

    def __init__(self, *, runtime: object, flow: DemoMotionFlow) -> None:
        self.runtime = runtime
        self.flow = flow

    def startup(self) -> None:
        self.runtime.open()
        try:
            self.flow.startup()
        except BaseException as startup_exc:
            stop_error: BaseException | None = None
            try:
                # transport 仍打开时先尝试停止；关闭后已无法发送 hold/stop。
                self.flow.stop()
            except BaseException as exc:
                stop_error = exc
            stop_report = getattr(self.flow, "last_stop_report", None)
            if stop_report is not None:
                try:
                    setattr(startup_exc, "stop_report", stop_report)
                except (AttributeError, TypeError):
                    pass
            try:
                self.runtime.close()
            finally:
                if stop_error is not None:
                    startup_exc.add_note(
                        f"startup compensation stop failed: {stop_error}"
                    )
            raise

    def require_base_motion_ready(self) -> None:
        self.flow.require_base_motion_ready()

    def plan_to_base_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float | None,
    ) -> object:
        return self.flow.plan_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)

    def execute_base_plan(self, plan: object) -> object:
        return self.flow.execute_plan(plan)

    @property
    def last_stop_report(self) -> object | None:
        return getattr(self.flow, "last_stop_report", None)

    def return_to_startup(self) -> object:
        return self.flow.return_to_startup()

    def stop(self) -> None:
        self.flow.stop()

    def enable_joints(self) -> None:
        self.flow.joints_command("enable")

    def disable_joints(self) -> None:
        self.flow.joints_command("disable")

    def suction_grip(self) -> None:
        self.flow.suction_command("grip")

    def suction_release(self) -> None:
        self.flow.suction_command("release")

    def suction_idle(self) -> None:
        self.flow.suction_command("idle")

    def get_status(self) -> None:
        self.flow.status()

    def capture_state(self) -> tuple[RobotAxisState, CaptureMotionState]:
        self.flow.require_base_motion_ready()
        return self.flow._planning_state(), CaptureMotionState.STATIONARY

    def plan_base_sequence(self, targets: tuple[object, ...]) -> tuple[object, ...]:
        from application.motion_target import BaseToolTarget
        from scripts.run_motion_demo import DemoStage

        self.flow.require_base_motion_ready()
        state = self.flow._planning_state()
        plans: list[tuple[DemoStage, ...]] = []
        for index, target in enumerate(targets):
            if not isinstance(target, BaseToolTarget):
                raise TypeError("sequence targets must be BaseToolTarget")
            if index == 0:
                stages = self.flow.plan_to_base_pose(
                    target.x_mm, target.y_mm, target.z_mm, target.yaw_deg
                )
            else:
                current_pose = self.flow.solver.forward_kinematics_base(state)
                transform = RigidTransform.from_xyz_yaw_deg(
                    x_mm=target.x_mm, y_mm=target.y_mm, z_mm=target.z_mm,
                    yaw_deg=current_pose.yaw_deg if target.yaw_deg is None else target.yaw_deg,
                )
                base_plan = self.flow.planner.plan(
                    current_state=state, base_T_tool_target=transform
                )
                stages = tuple(
                    DemoStage(
                        stage.kind.name,
                        stage.base_T_tool_target,
                        stage.multi_axis_target,
                        stage.solution,
                    )
                    for stage in base_plan.stages
                )
                for stage in stages:
                    self.runtime.controller.validate_positions(stage.multi_axis_target)
            if not stages or stages[-1].solution is None:
                raise RuntimeError("planned stage sequence has no final axis solution")
            plans.append(stages)
            state = stages[-1].solution.axis_state()
        return tuple(plans)

    def joints_holding(self) -> bool:
        return bool(self.runtime.controller.rotary_joints_enabled())

    def shutdown(self) -> None:
        self.runtime.close()


def create_mushroom_robot_controller(
    *,
    execute: bool,
    runtime_config: RobotRuntimeConfig,
    arm_local_workspace_config: ArmLocalWorkspaceConfig = (
        DEFAULT_ARM_LOCAL_WORKSPACE_CONFIG
    ),
    motion_envelope: RobotMotionEnvelopeConfig = (
        DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG
    ),
    emit: Callable[[str], None] = print,
) -> MushroomRobotController:
    """构造不打开硬件；只有随后显式 ``startup()`` 才进入既有流程。"""

    from scripts.run_motion_demo import create_demo_flow

    workspace = TrayWorkspace(runtime_config.tray_workspace)
    runtime, flow = create_demo_flow(
        execute=execute,
        arm_local_workspace_config=arm_local_workspace_config,
        motion_envelope=motion_envelope,
        emit=emit,
    )
    calibration = hand_eye_from_frame_document(
        runtime_config.frame_transforms,
        source=f"{runtime_config.source_path}#frame_transforms",
    )
    resolver = VisionTargetResolver(
        pose_provider=flow.solver,
        hand_eye_calibration=calibration,
    )
    return MushroomRobotController(
        base_backend=DemoFlowApplicationBackend(runtime=runtime, flow=flow),
        tray_workspace=workspace,
        target_resolver=resolver,
    )


__all__ = [
    "DemoFlowApplicationBackend",
    "create_mushroom_robot_controller",
]
