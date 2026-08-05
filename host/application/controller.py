"""蘑菇机器人应用层唯一入口及能力门禁。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

from application.tray_workspace import TrayWorkspace
from application.motion_target import BaseToolTarget
from calibration.hand_eye import HandEyeCalibrationStatus, hand_eye_status
from geometry.rigid_transform import RigidTransform
from vision.observation import VisionTargetObservation
from vision.target_resolver import (
    HandEyeCalibrationUnavailable,
    VisionTargetResolver,
)


class UnsupportedToolGoalOrientationError(ValueError):
    """解析结果超出当前仅支持 xyz+yaw 的 Base 目标接口。"""


@runtime_checkable
class BaseFrameRobotBackend(Protocol):
    """已经验证的 Base-frame 运动链所需的最小适配面。"""

    def startup(self) -> object: ...

    def require_base_motion_ready(self) -> None: ...

    def plan_to_base_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float | None,
    ) -> object: ...

    def execute_base_plan(self, plan: object) -> object: ...

    def return_to_startup(self) -> object: ...

    def stop(self) -> object: ...

    def enable_joints(self) -> object: ...

    def disable_joints(self) -> object: ...

    def suction_grip(self) -> object: ...

    def suction_release(self) -> object: ...

    def get_status(self) -> object: ...

    def shutdown(self) -> object: ...


@dataclass(frozen=True)
class RobotCapabilities:
    base_frame_motion: bool
    suction_control: bool
    rotary_joint_enable_control: bool
    hand_eye_calibration: HandEyeCalibrationStatus
    vision_target_resolution: bool
    vision_target_motion: bool


@dataclass(frozen=True)
class MushroomRobotStatus:
    capabilities: RobotCapabilities
    backend_status: object


class MushroomRobotController:
    """Base 运动可用、Camera 运动默认受门禁的应用层 façade。"""

    def __init__(
        self,
        *,
        base_backend: BaseFrameRobotBackend,
        tray_workspace: TrayWorkspace,
        target_resolver: VisionTargetResolver | None = None,
        suction_control: bool = True,
        rotary_joint_enable_control: bool = True,
        orientation_tolerance_deg: float = 1e-6,
    ) -> None:
        if not isinstance(base_backend, BaseFrameRobotBackend):
            raise TypeError("base_backend must implement BaseFrameRobotBackend")
        if not isinstance(tray_workspace, TrayWorkspace):
            raise TypeError("tray_workspace must be TrayWorkspace")
        if target_resolver is not None and not isinstance(
            target_resolver, VisionTargetResolver
        ):
            raise TypeError("target_resolver must be VisionTargetResolver or None")
        if not isinstance(suction_control, bool):
            raise TypeError("suction_control must be a bool")
        if not isinstance(rotary_joint_enable_control, bool):
            raise TypeError("rotary_joint_enable_control must be a bool")
        if (
            isinstance(orientation_tolerance_deg, bool)
            or not isinstance(orientation_tolerance_deg, (int, float))
            or not math.isfinite(orientation_tolerance_deg)
            or orientation_tolerance_deg < 0.0
        ):
            raise ValueError(
                "orientation_tolerance_deg must be finite and non-negative"
            )
        self._base_backend = base_backend
        self._tray_workspace = tray_workspace
        self._target_resolver = target_resolver
        self._suction_control = suction_control
        self._rotary_joint_enable_control = rotary_joint_enable_control
        self._orientation_tolerance_deg = float(orientation_tolerance_deg)

    @property
    def capabilities(self) -> RobotCapabilities:
        resolver = self._target_resolver
        calibration = None if resolver is None else resolver.hand_eye_calibration
        resolution_available = resolver is not None and resolver.available
        return RobotCapabilities(
            base_frame_motion=True,
            suction_control=self._suction_control,
            rotary_joint_enable_control=self._rotary_joint_enable_control,
            hand_eye_calibration=hand_eye_status(calibration),
            vision_target_resolution=resolution_available,
            vision_target_motion=resolution_available,
        )

    @property
    def tray_workspace(self) -> TrayWorkspace:
        return self._tray_workspace

    @property
    def target_resolver(self) -> VisionTargetResolver | None:
        return self._target_resolver

    def startup(self) -> object:
        return self._base_backend.startup()

    def plan_to_base_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float | None = None,
    ) -> object:
        return self.plan_base_target(
            BaseToolTarget(x_mm, y_mm, z_mm, yaw_deg),
            enforce_tray_workspace=True,
        )

    def plan_base_target(
        self,
        target: BaseToolTarget,
        *,
        enforce_tray_workspace: bool = True,
    ) -> object:
        """复用唯一 Base 规划出口；仅抓取中间高位阶段可跳过 Tray 门限。"""

        if not isinstance(target, BaseToolTarget):
            raise TypeError("target must be a BaseToolTarget")
        if not isinstance(enforce_tray_workspace, bool):
            raise TypeError("enforce_tray_workspace must be a bool")
        self._base_backend.require_base_motion_ready()
        if enforce_tray_workspace:
            self._tray_workspace.require_xyz_allowed(
                x_mm=target.x_mm,
                y_mm=target.y_mm,
                z_mm=target.z_mm,
            )
        return self._base_backend.plan_to_base_pose(
            target.x_mm, target.y_mm, target.z_mm, target.yaw_deg
        )

    def plan_base_target_sequence(
        self,
        targets: tuple[BaseToolTarget, ...],
        *,
        enforce_tray_workspace: tuple[bool, ...],
    ) -> tuple[object, ...]:
        """按前一目标终态串接纯规划；任一阶段失败则不返回部分结果。"""

        if not isinstance(targets, tuple) or not targets or not all(
            isinstance(target, BaseToolTarget) for target in targets
        ):
            raise TypeError("targets must be a non-empty tuple of BaseToolTarget")
        if (
            not isinstance(enforce_tray_workspace, tuple)
            or len(enforce_tray_workspace) != len(targets)
            or not all(isinstance(value, bool) for value in enforce_tray_workspace)
        ):
            raise TypeError("enforce_tray_workspace must be a matching bool tuple")
        self._base_backend.require_base_motion_ready()
        for target, enforce in zip(targets, enforce_tray_workspace, strict=True):
            if enforce:
                self._tray_workspace.require_xyz_allowed(
                    x_mm=target.x_mm, y_mm=target.y_mm, z_mm=target.z_mm
                )
        method = getattr(self._base_backend, "plan_base_sequence", None)
        if method is not None:
            plans = method(targets)
            if not isinstance(plans, tuple) or len(plans) != len(targets):
                raise RuntimeError("backend returned an invalid Base plan sequence")
            return plans
        # 兼容只实现原最小 Protocol 的测试/外部后端；仍保证只规划、不执行。
        return tuple(
            self._base_backend.plan_to_base_pose(
                target.x_mm, target.y_mm, target.z_mm, target.yaw_deg
            )
            for target in targets
        )

    def move_to_base_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float | None = None,
    ) -> object:
        plan = self.plan_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)
        return self._base_backend.execute_base_plan(plan)

    def execute_base_plan(self, plan: object) -> object:
        return self._base_backend.execute_base_plan(plan)

    def resolve_object_in_base(
        self,
        observation: VisionTargetObservation,
    ) -> RigidTransform:
        if self._target_resolver is None:
            raise HandEyeCalibrationUnavailable(
                "Hand-eye calibration is missing or not validated."
            )
        return self._target_resolver.resolve_object_in_base(observation)

    def plan_to_observation(
        self,
        observation: VisionTargetObservation,
        grasp_offset: RigidTransform,
    ) -> object:
        target = self._resolve_tool_goal(observation, grasp_offset)
        x_mm, y_mm, z_mm, yaw_deg = self._base_pose_arguments(target)
        return self.plan_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)

    def move_to_observation(
        self,
        observation: VisionTargetObservation,
        grasp_offset: RigidTransform,
    ) -> object:
        target = self._resolve_tool_goal(observation, grasp_offset)
        x_mm, y_mm, z_mm, yaw_deg = self._base_pose_arguments(target)
        return self.move_to_base_pose(x_mm, y_mm, z_mm, yaw_deg)

    def return_to_startup(self) -> object:
        return self._base_backend.return_to_startup()

    def stop(self) -> object:
        return self._base_backend.stop()

    def enable_joints(self) -> object:
        return self._base_backend.enable_joints()

    def disable_joints(self) -> object:
        return self._base_backend.disable_joints()

    def suction_grip(self) -> object:
        return self._base_backend.suction_grip()

    def suction_release(self) -> object:
        return self._base_backend.suction_release()

    def suction_idle(self) -> object:
        method = getattr(self._base_backend, "suction_idle", None)
        if method is None:
            raise RuntimeError("backend does not expose suction_idle")
        return method()

    def get_status(self) -> MushroomRobotStatus:
        return MushroomRobotStatus(
            capabilities=self.capabilities,
            backend_status=self._base_backend.get_status(),
        )

    def shutdown(self) -> object:
        return self._base_backend.shutdown()

    def _resolve_tool_goal(
        self,
        observation: VisionTargetObservation,
        grasp_offset: RigidTransform,
    ) -> RigidTransform:
        if self._target_resolver is None:
            raise HandEyeCalibrationUnavailable(
                "Hand-eye calibration is missing or not validated. "
                "Base-frame manual motion remains available. "
                "Camera-target motion is disabled."
            )
        return self._target_resolver.resolve_tool_goal_in_base(
            observation,
            grasp_offset,
        )

    def _base_pose_arguments(
        self,
        target: RigidTransform,
    ) -> tuple[float, float, float, float]:
        roll_deg, pitch_deg, yaw_deg = (float(value) for value in target.rpy_deg)
        tolerance = self._orientation_tolerance_deg
        if abs(roll_deg) > tolerance or abs(pitch_deg) > tolerance:
            raise UnsupportedToolGoalOrientationError(
                "resolved base_T_tool_goal contains roll/pitch outside the current "
                "xyz+yaw Base-frame motion contract: "
                f"roll={roll_deg:.9g} deg, pitch={pitch_deg:.9g} deg"
            )
        x_mm, y_mm, z_mm = (float(value) for value in target.translation_mm)
        return x_mm, y_mm, z_mm, yaw_deg


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


__all__ = [
    "BaseFrameRobotBackend",
    "BaseToolTarget",
    "MushroomRobotController",
    "MushroomRobotStatus",
    "RobotCapabilities",
    "UnsupportedToolGoalOrientationError",
]
