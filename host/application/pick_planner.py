"""把已验证视觉观察转换为完整、原子的三阶段抓取计划。"""

from __future__ import annotations

from dataclasses import dataclass
import time

from application.controller import MushroomRobotController, UnsupportedToolGoalOrientationError
from application.grasp_profile import GraspProfile, GraspYawMode
from application.motion_target import BaseToolTarget
from config.project.robot_motion_envelope import WORKING_HEIGHT_BASE_Z_MM
from vision.observation import VisionTargetObservation


class PickPlanningError(ValueError):
    pass


class ObservationConfidenceError(PickPlanningError):
    pass


class ObservationStaleError(PickPlanningError):
    pass


class ObservationOrientationUnavailable(PickPlanningError):
    pass


@dataclass(frozen=True)
class PickPlan:
    observation: VisionTargetObservation
    overhead_target: BaseToolTarget
    contact_target: BaseToolTarget
    lift_target: BaseToolTarget
    overhead_motion: object
    contact_motion: object
    lift_motion: object
    suction_settle_time_s: float


class PickPlanner:
    def __init__(self, controller: MushroomRobotController, *, clock=time.time, orientation_tolerance_deg: float = 1e-6) -> None:
        if not isinstance(controller, MushroomRobotController):
            raise TypeError("controller must be a MushroomRobotController")
        self.controller = controller
        self.clock = clock
        self.orientation_tolerance_deg = float(orientation_tolerance_deg)

    def plan(self, observation: VisionTargetObservation, grasp_profile: GraspProfile) -> PickPlan:
        if not isinstance(observation, VisionTargetObservation):
            raise TypeError("observation must be a VisionTargetObservation")
        if not isinstance(grasp_profile, GraspProfile):
            raise TypeError("grasp_profile must be a GraspProfile")
        self._validate_quality(observation, grasp_profile)
        object_pose = self.controller.resolve_object_in_base(observation)
        yaw = self._select_yaw(observation, grasp_profile, object_pose)
        x_mm, y_mm, object_z_mm = (float(value) for value in object_pose.translation_mm)
        overhead = BaseToolTarget(x_mm, y_mm, WORKING_HEIGHT_BASE_Z_MM, yaw)
        contact = BaseToolTarget(x_mm, y_mm, object_z_mm + grasp_profile.contact_offset_mm, yaw)
        if contact.z_mm >= WORKING_HEIGHT_BASE_Z_MM:
            raise PickPlanningError(
                f"contact Base Z {contact.z_mm:g} mm must be below working height "
                f"{WORKING_HEIGHT_BASE_Z_MM:g} mm"
            )
        lift = BaseToolTarget(x_mm, y_mm, WORKING_HEIGHT_BASE_Z_MM, yaw)

        # 三个阶段必须全部规划成功后才构造 PickPlan。只有 contact 是 Tray 最终任务目标。
        overhead_motion, contact_motion, lift_motion = self.controller.plan_base_target_sequence(
            (overhead, contact, lift),
            enforce_tray_workspace=(False, True, False),
        )
        return PickPlan(
            observation,
            overhead,
            contact,
            lift,
            overhead_motion,
            contact_motion,
            lift_motion,
            grasp_profile.suction_settle_time_s,
        )

    def _validate_quality(self, observation: VisionTargetObservation, profile: GraspProfile) -> None:
        if observation.confidence is None or observation.confidence < profile.minimum_confidence:
            raise ObservationConfidenceError(
                f"observation confidence {observation.confidence!r} is below required {profile.minimum_confidence}"
            )
        if observation.timestamp is None:
            raise ObservationStaleError("observation timestamp is required for pick planning")
        age = float(self.clock()) - observation.timestamp
        if abs(age) > profile.maximum_observation_age_s:
            raise ObservationStaleError(
                f"observation age {age:g}s is outside "
                f"[-{profile.maximum_observation_age_s:g}, "
                f"{profile.maximum_observation_age_s:g}]s"
            )

    def _select_yaw(self, observation: VisionTargetObservation, profile: GraspProfile, object_pose: object) -> float:
        if profile.yaw_mode is GraspYawMode.FIXED:
            assert profile.fixed_yaw_deg is not None
            return float(profile.fixed_yaw_deg)
        if profile.yaw_mode is GraspYawMode.KEEP_CURRENT:
            resolver = self.controller.target_resolver
            if resolver is None:
                raise PickPlanningError("vision target resolver is unavailable")
            return resolver.pose_provider.forward_kinematics_base(observation.capture_axis_state).yaw_deg
        if observation.orientation is None:
            raise ObservationOrientationUnavailable(
                "FROM_OBSERVATION yaw requires a non-null observation orientation"
            )
        roll_deg, pitch_deg, yaw_deg = (float(value) for value in object_pose.rpy_deg)
        if abs(roll_deg) > self.orientation_tolerance_deg or abs(pitch_deg) > self.orientation_tolerance_deg:
            raise UnsupportedToolGoalOrientationError(
                "observed target orientation has non-zero roll/pitch outside the xyz+yaw contract"
            )
        return yaw_deg


__all__ = [
    "BaseToolTarget", "ObservationConfidenceError", "ObservationOrientationUnavailable",
    "ObservationStaleError", "PickPlan", "PickPlanner", "PickPlanningError",
]
