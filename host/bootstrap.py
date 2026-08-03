"""Side-effect-free assembly of in-process motion client interfaces."""

from __future__ import annotations

from typing import TYPE_CHECKING

from motion.client_facades import FrontendMotionFacade, KinematicsMotionFacade
from motion.client_interfaces import (
    FrontendMotionInterface,
    KinematicsMotionInterface,
)

if TYPE_CHECKING:
    from motion.unified_controller import UnifiedMotionController


class UpperMotionRuntime:
    """Expose two client views over one caller-owned unified controller.

    Construction performs no hardware I/O. Hardware lifecycle remains the
    responsibility of the program entry point that created the controller and
    its backends.
    """

    def __init__(self, controller: UnifiedMotionController) -> None:
        self._controller = controller
        self._frontend_motion: FrontendMotionInterface = FrontendMotionFacade(
            controller
        )
        self._kinematics_motion: KinematicsMotionInterface = (
            KinematicsMotionFacade(controller)
        )

    @property
    def frontend_motion(self) -> FrontendMotionInterface:
        return self._frontend_motion

    @property
    def kinematics_motion(self) -> KinematicsMotionInterface:
        return self._kinematics_motion


def create_upper_motion_runtime(
    controller: UnifiedMotionController,
) -> UpperMotionRuntime:
    """Create the two stable client views without constructing more hardware."""

    return UpperMotionRuntime(controller)


__all__ = ["UpperMotionRuntime", "create_upper_motion_runtime"]
