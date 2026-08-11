"""经手眼标定门禁的 Camera-to-Base 目标解析边界。"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from calibration.hand_eye import HandEyeCalibration
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from vision.observation import CaptureMotionState, VisionTargetObservation


_UNAVAILABLE_MESSAGE = (
    "Hand-eye calibration is missing or not validated. "
    "Base-frame manual motion remains available. "
    "Camera-target motion is disabled."
)


class VisionTargetResolutionError(RuntimeError):
    """视觉观察不能安全解析为 Base frame 目标。"""


class HandEyeCalibrationUnavailable(VisionTargetResolutionError):
    """手眼外参缺失或尚未验证。"""


class CaptureStateUnavailable(VisionTargetResolutionError):
    """采集时机器人并非已确认静止。"""


class ObservationFrameMismatch(VisionTargetResolutionError):
    """观察 frame 与解析器约定不一致。"""


@runtime_checkable
class BaseToolPoseProvider(Protocol):
    def forward_kinematics_base(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform: ...


class VisionTargetResolver:
    """只组合变换，不规划、不访问硬件、不执行运动。"""

    def __init__(
        self,
        *,
        pose_provider: BaseToolPoseProvider,
        hand_eye_calibration: HandEyeCalibration | None,
        camera_frame_id: str = "camera",
    ) -> None:
        if not isinstance(pose_provider, BaseToolPoseProvider):
            raise TypeError(
                "pose_provider must implement forward_kinematics_base(RobotAxisState)"
            )
        if hand_eye_calibration is not None and not isinstance(
            hand_eye_calibration, HandEyeCalibration
        ):
            raise TypeError(
                "hand_eye_calibration must be HandEyeCalibration or None"
            )
        if not isinstance(camera_frame_id, str) or not camera_frame_id.strip():
            raise ValueError("camera_frame_id must be a non-empty string")
        self.pose_provider = pose_provider
        self.hand_eye_calibration = hand_eye_calibration
        self.camera_frame_id = camera_frame_id

    @property
    def available(self) -> bool:
        calibration = self.hand_eye_calibration
        return calibration is not None and calibration.validated

    def resolve_object_in_base(
        self,
        observation: VisionTargetObservation,
    ) -> RigidTransform:
        self._validate_observation(observation)
        assert self.hand_eye_calibration is not None
        base_T_tool_capture = self.pose_provider.forward_kinematics_base(
            observation.capture_axis_state
        )
        if not isinstance(base_T_tool_capture, RigidTransform):
            raise VisionTargetResolutionError(
                "pose provider returned a non-RigidTransform base_T_tool"
            )
        raw_base_T_object = (
            base_T_tool_capture
            @ self.hand_eye_calibration.tool_T_camera
            @ observation.camera_T_target
        )
        return self.hand_eye_calibration.compensate_base_pose(raw_base_T_object)

    def resolve_tool_goal_in_base(
        self,
        observation: VisionTargetObservation,
        grasp_offset: RigidTransform,
    ) -> RigidTransform:
        if not isinstance(grasp_offset, RigidTransform):
            raise TypeError("grasp_offset must be a RigidTransform")
        return self.resolve_object_in_base(observation) @ grasp_offset

    def _validate_observation(self, observation: VisionTargetObservation) -> None:
        if not self.available:
            raise HandEyeCalibrationUnavailable(_UNAVAILABLE_MESSAGE)
        if not isinstance(observation, VisionTargetObservation):
            raise TypeError("observation must be a VisionTargetObservation")
        if observation.frame_id != self.camera_frame_id:
            raise ObservationFrameMismatch(
                f"observation frame_id={observation.frame_id!r} does not match "
                f"camera frame {self.camera_frame_id!r}"
            )
        if observation.capture_motion_state is not CaptureMotionState.STATIONARY:
            raise CaptureStateUnavailable(
                "camera observation requires a stationary, arrived five-axis "
                "capture state; moving or unknown capture state is unsupported"
            )


__all__ = [
    "BaseToolPoseProvider",
    "CaptureStateUnavailable",
    "HandEyeCalibrationUnavailable",
    "ObservationFrameMismatch",
    "VisionTargetResolutionError",
    "VisionTargetResolver",
]
