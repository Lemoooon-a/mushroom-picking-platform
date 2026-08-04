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
    load_five_axis_geometry,
    load_local_five_axis_kinematics,
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
    UnvalidatedBaseTransformError,
)


__all__ = [
    "BaseFrameFiveAxisSolver",
    "BaseFrameSolverConfig",
    "BaseFrameSolverError",
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
    "PlanarPoint",
    "RobotAxisState",
    "RobotFrameChain",
    "SlideZeroKinematics",
    "SolverWeights",
    "UnvalidatedBaseTransformError",
    "UnreachableTargetError",
    "load_five_axis_geometry",
    "load_local_five_axis_kinematics",
    "rotation_deg_for_output_yaw",
    "rotation_output_yaw_deg",
]
