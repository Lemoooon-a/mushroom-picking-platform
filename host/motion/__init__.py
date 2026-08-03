"""统一异步点到点运动控制；硬件相关控制器按需延迟导入。"""

from motion.client_facades import FrontendMotionFacade, KinematicsMotionFacade
from motion.client_interfaces import (
    FrontendMotionInterface,
    KinematicsMotionInterface,
)
from motion.unified_protocol import (
    ArrivalConfig,
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
)

__all__ = [
    "ArrivalConfig",
    "AxisCapabilities",
    "AxisDescriptor",
    "AxisKind",
    "AxisName",
    "AxisState",
    "AxisTarget",
    "FrontendMotionFacade",
    "FrontendMotionInterface",
    "KinematicsMotionFacade",
    "KinematicsMotionInterface",
    "MotionCommandHandle",
    "MotionCommandResult",
    "MotionCommandStatus",
    "MotionErrorCode",
    "MultiAxisCommandHandle",
    "MultiAxisCommandResult",
    "MultiAxisSubmissionError",
    "MultiAxisTarget",
    "UnifiedMotionController",
    "UnifiedMotionError",
]


def __getattr__(name: str) -> object:
    if name in {
        "MultiAxisSubmissionError",
        "UnifiedMotionController",
        "UnifiedMotionError",
    }:
        from motion import unified_controller

        return getattr(unified_controller, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
