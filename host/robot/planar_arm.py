"""将平面二连杆逆运动学解映射到已标定的肩、肘关节。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from kinematics import JointAngles, Planar2RKinematics, PlanarPoint
from robot.joint import JointConfig, JointState


_LIMIT_TOLERANCE_RAD = 1e-12
_SINGULAR_BRANCH_TOLERANCE = 1e-12


class PlanarArmError(Exception):
    """平面二关节调用层基础异常。"""


class NoJointLimitSolutionError(PlanarArmError):
    """数学逆解无法同时满足肩、肘软限位或指定分支。"""


class PlanarArmCommandError(PlanarArmError):
    """双关节下发未全部完成，已尝试软件停止。"""


@dataclass(frozen=True)
class PlanarArmTarget:
    """通过关节软限位筛选的末端目标和肩肘角。"""

    point: PlanarPoint
    angles: JointAngles


class PositionJoint(Protocol):
    """双关节调用层所需的最小关节接口。"""

    config: JointConfig

    def validate_position_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
    ) -> None: ...

    def command_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
    ) -> JointState: ...

    def stop(self) -> None: ...


def joint_limited_solutions(
    kinematics: Planar2RKinematics,
    shoulder_config: JointConfig,
    elbow_config: JointConfig,
    x: float,
    y: float,
) -> tuple[JointAngles, ...]:
    """返回同时满足肩、肘逻辑软限位的逆运动学解。"""

    return tuple(
        solution
        for solution in kinematics.inverse(x, y)
        if _within_limits(solution.shoulder_rad, shoulder_config)
        and _within_limits(solution.elbow_rad, elbow_config)
    )


def select_joint_target(
    kinematics: Planar2RKinematics,
    shoulder_config: JointConfig,
    elbow_config: JointConfig,
    x: float,
    y: float,
    *,
    elbow_branch: str = "positive",
) -> PlanarArmTarget:
    """按肘角正支或负支选择一个软限位内的目标。"""

    if elbow_branch not in ("positive", "negative"):
        raise ValueError(
            "elbow_branch must be 'positive' or 'negative', "
            f"got {elbow_branch!r}"
        )
    mathematical_solutions = kinematics.inverse(x, y)
    valid_solutions = tuple(
        solution
        for solution in mathematical_solutions
        if _within_limits(solution.shoulder_rad, shoulder_config)
        and _within_limits(solution.elbow_rad, elbow_config)
    )
    branch_solutions = tuple(
        solution
        for solution in valid_solutions
        if _matches_elbow_branch(solution.elbow_rad, elbow_branch)
    )
    if not branch_solutions:
        rendered = ", ".join(
            "(shoulder={:.3f} deg, elbow={:.3f} deg)".format(
                math.degrees(solution.shoulder_rad),
                math.degrees(solution.elbow_rad),
            )
            for solution in mathematical_solutions
        )
        raise NoJointLimitSolutionError(
            f"target ({x:.6g}, {y:.6g}) has no {elbow_branch} elbow solution "
            "inside shoulder [{:.3f}, {:.3f}] deg and elbow "
            "[{:.3f}, {:.3f}] deg limits; mathematical solutions: {}".format(
                math.degrees(shoulder_config.min_position_rad),
                math.degrees(shoulder_config.max_position_rad),
                math.degrees(elbow_config.min_position_rad),
                math.degrees(elbow_config.max_position_rad),
                rendered or "none",
            )
        )
    return PlanarArmTarget(
        point=PlanarPoint(x=x, y=y),
        angles=branch_solutions[0],
    )


class Planar2RArmController:
    """使用共享 CAN 总线的肩肘双关节位置调用层。

    两条 0xA4 指令依次快速发送，属于背靠背下发，不是硬件时钟同步。
    """

    def __init__(
        self,
        kinematics: Planar2RKinematics,
        shoulder: PositionJoint,
        elbow: PositionJoint,
    ) -> None:
        self.kinematics = kinematics
        self.shoulder = shoulder
        self.elbow = elbow

    def plan_target(
        self,
        x: float,
        y: float,
        *,
        elbow_branch: str = "positive",
    ) -> PlanarArmTarget:
        return select_joint_target(
            self.kinematics,
            self.shoulder.config,
            self.elbow.config,
            x,
            y,
            elbow_branch=elbow_branch,
        )

    def command_target(
        self,
        target: PlanarArmTarget,
        *,
        shoulder_velocity_rad_s: float,
        elbow_velocity_rad_s: float,
    ) -> tuple[JointState, JointState]:
        """先离线验证两关节，再背靠背下发位置命令。"""

        angles = target.angles
        self.shoulder.validate_position_command(
            angles.shoulder_rad,
            shoulder_velocity_rad_s,
        )
        self.elbow.validate_position_command(
            angles.elbow_rad,
            elbow_velocity_rad_s,
        )

        try:
            shoulder_state = self.shoulder.command_position(
                angles.shoulder_rad,
                shoulder_velocity_rad_s,
            )
            elbow_state = self.elbow.command_position(
                angles.elbow_rad,
                elbow_velocity_rad_s,
            )
        except Exception as command_error:
            stop_errors: list[str] = []
            for joint in (self.shoulder, self.elbow):
                try:
                    joint.stop()
                except Exception as stop_error:
                    stop_errors.append(
                        f"{joint.config.name}: {stop_error}"
                    )
            stop_detail = (
                "; stop errors: " + "; ".join(stop_errors)
                if stop_errors
                else "; both current-position hold commands were accepted"
            )
            raise PlanarArmCommandError(
                f"dual-joint position submission did not complete: {command_error}"
                + stop_detail
            ) from command_error
        return shoulder_state, elbow_state


def _within_limits(position_rad: float, config: JointConfig) -> bool:
    return (
        config.min_position_rad - _LIMIT_TOLERANCE_RAD
        <= position_rad
        <= config.max_position_rad + _LIMIT_TOLERANCE_RAD
    )


def _matches_elbow_branch(elbow_rad: float, branch: str) -> bool:
    if abs(math.sin(elbow_rad)) <= _SINGULAR_BRANCH_TOLERANCE:
        return True
    return elbow_rad > 0.0 if branch == "positive" else elbow_rad < 0.0


__all__ = [
    "NoJointLimitSolutionError",
    "Planar2RArmController",
    "PlanarArmCommandError",
    "PlanarArmError",
    "PlanarArmTarget",
    "joint_limited_solutions",
    "select_joint_target",
]
