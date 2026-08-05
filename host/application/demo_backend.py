"""把现有 DemoMotionFlow 薄适配为应用层 Base-frame 后端。"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

from application.controller import MushroomRobotController
from application.tray_workspace import TrayWorkspace
from calibration.hand_eye import hand_eye_from_frame_document
from config.frame_transforms import load_frame_transforms_document
from config.project.robot_motion_envelope import (
    DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG,
    RobotMotionEnvelopeConfig,
)
from config.tray_workspace import load_tray_workspace_config
from config.project.workspace_planning import (
    DEFAULT_OFFSET_WORKSPACE_CONFIG,
    OffsetWorkspaceConfig,
)
from vision.target_resolver import VisionTargetResolver

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
        except BaseException:
            self.runtime.close()
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

    def get_status(self) -> None:
        self.flow.status()

    def shutdown(self) -> None:
        self.runtime.close()


def create_mushroom_robot_controller(
    *,
    execute: bool,
    frame_config: Path,
    tray_workspace_config: Path,
    offset_workspace_config: OffsetWorkspaceConfig = DEFAULT_OFFSET_WORKSPACE_CONFIG,
    motion_envelope: RobotMotionEnvelopeConfig = (
        DEFAULT_ROBOT_MOTION_ENVELOPE_CONFIG
    ),
    emit: Callable[[str], None] = print,
) -> MushroomRobotController:
    """构造不打开硬件；只有随后显式 ``startup()`` 才进入既有流程。"""

    from scripts.run_motion_demo import create_demo_flow

    workspace = TrayWorkspace(load_tray_workspace_config(tray_workspace_config))
    runtime, flow = create_demo_flow(
        execute=execute,
        frame_config=frame_config,
        offset_workspace_config=offset_workspace_config,
        motion_envelope=motion_envelope,
        emit=emit,
    )
    document = load_frame_transforms_document(frame_config)
    calibration = hand_eye_from_frame_document(
        document,
        source=str(frame_config),
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


__all__ = ["DemoFlowApplicationBackend", "create_mushroom_robot_controller"]
