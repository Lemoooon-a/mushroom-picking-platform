"""机器人运动学与坐标变换层。"""

from .planar_2r import (
    JointAngles,
    KinematicsError,
    Planar2RKinematics,
    PlanarPoint,
    UnreachableTargetError,
)
from .frame_chain import (
    FrameChainError,
    MissingToolCameraTransformError,
    RobotAxisState,
    RobotFrameChain,
    SlideZeroKinematics,
)
from .five_axis import (
    FiveAxisGeometry,
    FiveAxisGeometryError,
    FiveAxisKinematics,
    PlanarLocalTarget,
    load_five_axis_geometry,
    load_robot_five_axis_kinematics,
    rotation_deg_for_output_yaw,
    rotation_output_yaw_deg,
)
from .base_frame_solver import (
    BaseFrameFiveAxisSolver,
    BaseFrameSolverConfig,
    BaseFrameSolverError,
    FiveAxisNoSolutionError,
    FiveAxisSolution,
    SolverWeights,
)
from .base_move_transition_planner import (
    BaseMovePlan,
    BaseMovePlanningError,
    BaseMoveStage,
    BaseMoveStageKind,
    BaseMoveTransitionPlanner,
    ClearanceHeightUnreachableError,
    CurrentStateInvalidError,
    StageValidationFailedError,
)


__all__ = [
    "BaseFrameFiveAxisSolver",
    "BaseFrameSolverConfig",
    "BaseFrameSolverError",
    "BaseMovePlan",
    "BaseMovePlanningError",
    "BaseMoveStage",
    "BaseMoveStageKind",
    "BaseMoveTransitionPlanner",
    "ClearanceHeightUnreachableError",
    "CurrentStateInvalidError",
    "FrameChainError",
    "FiveAxisGeometry",
    "FiveAxisGeometryError",
    "FiveAxisKinematics",
    "FiveAxisNoSolutionError",
    "FiveAxisSolution",
    "JointAngles",
    "KinematicsError",
    "MissingToolCameraTransformError",
    "Planar2RKinematics",
    "PlanarLocalTarget",
    "PlanarPoint",
    "RobotAxisState",
    "RobotFrameChain",
    "SlideZeroKinematics",
    "SolverWeights",
    "StageValidationFailedError",
    "UnreachableTargetError",
    "load_five_axis_geometry",
    "load_robot_five_axis_kinematics",
    "rotation_deg_for_output_yaw",
    "rotation_output_yaw_deg",
]
