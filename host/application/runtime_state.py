"""Robot Service 的简单进程级状态。"""

from enum import Enum


class RobotServiceState(str, Enum):
    CREATED = "created"
    STARTING = "starting"
    READY = "ready"
    OBSERVING = "observing"
    PLANNING = "planning"
    EXECUTING = "executing"
    DISABLED = "disabled"
    FAULT = "fault"
    SHUTDOWN = "shutdown"


class RobotServiceMode(str, Enum):
    READ_ONLY = "read-only"
    DRY_RUN = "dry-run"
    EXECUTE = "execute"


__all__ = ["RobotServiceMode", "RobotServiceState"]
