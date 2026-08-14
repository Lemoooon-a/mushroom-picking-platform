"""机械 Base 根下的简化五轴正逆运动学。

位置链固定为：Slide + 平面 Shoulder/Elbow 2R + Z -> TCP；Rotation 只贡献
TCP yaw，不参与 TCP XYZ。连杆长度和 Z=0 时 TCP 的 Base 高度由本机 JSON
显式提供；模块没有机械尺寸默认值，也不读取 startup position 或硬件。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from kinematics.planar_2r import Planar2RKinematics


SCHEMA_VERSION = 1
DEFAULT_ROBOT_GEOMETRY_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "robot_geometry.json"
)


class FiveAxisGeometryError(ValueError):
    """五轴几何缺失、未经确认或违反模型约束。"""


@dataclass(frozen=True)
class PlanarLocalTarget:
    """固定 Slide 后，TCP 在肩肘机械平面中的局部目标。"""

    local_x_mm: float
    local_y_mm: float
    z_axis_mm: float
    tool_z_mm: float
    tool_yaw_deg: float


@dataclass(frozen=True)
class FiveAxisGeometry:
    """简化 FK 所需的纯机械几何。"""

    link1_length_mm: float
    link2_length_mm: float
    tcp_height_at_z_zero_mm: float

    def __post_init__(self) -> None:
        _require_positive("link1_length_mm", self.link1_length_mm)
        _require_positive("link2_length_mm", self.link2_length_mm)
        _require_finite("tcp_height_at_z_zero_mm", self.tcp_height_at_z_zero_mm)


class FiveAxisKinematics:
    """Base/TCP 简化模型的纯计算 FK provider。"""

    def __init__(self, geometry: FiveAxisGeometry) -> None:
        if not isinstance(geometry, FiveAxisGeometry):
            raise TypeError("geometry must be FiveAxisGeometry")
        self.geometry = geometry
        self.planar_2r = Planar2RKinematics(
            link1_length=geometry.link1_length_mm,
            link2_length=geometry.link2_length_mm,
        )

    def forward_kinematics(
        self,
        axis_state: RobotAxisState,
    ) -> RigidTransform:
        """返回 ``base_T_tool``；输入为统一逻辑 mm/deg 状态。"""

        if not isinstance(axis_state, RobotAxisState):
            raise TypeError("axis_state must be RobotAxisState")
        shoulder_rad = math.radians(axis_state.shoulder_deg)
        elbow_rad = math.radians(axis_state.elbow_deg)
        planar_point = self.planar_2r.forward(shoulder_rad, elbow_rad)

        output_yaw_deg = rotation_output_yaw_deg(
            axis_state.shoulder_deg,
            axis_state.elbow_deg,
            axis_state.rotation_deg,
        )
        return RigidTransform.from_xyz_yaw_deg(
            x_mm=planar_point.x,
            y_mm=axis_state.slide_mm + planar_point.y,
            z_mm=self.geometry.tcp_height_at_z_zero_mm + axis_state.z_mm,
            yaw_deg=output_yaw_deg,
        )

    def compute_arm_local_target(
        self,
        base_T_tool: RigidTransform,
        slide_mm: float,
    ) -> PlanarLocalTarget:
        """直接计算肩肘局部 XY 和对应 Z 轴逻辑位置。"""

        if not isinstance(base_T_tool, RigidTransform):
            raise TypeError("base_T_tool must be RigidTransform")
        slide = _require_finite("slide_mm", slide_mm)
        x_mm, y_mm, z_mm = (float(value) for value in base_T_tool.translation_mm)
        return PlanarLocalTarget(
            local_x_mm=x_mm,
            local_y_mm=y_mm - slide,
            z_axis_mm=z_mm - self.geometry.tcp_height_at_z_zero_mm,
            tool_z_mm=z_mm,
            tool_yaw_deg=float(base_T_tool.yaw_deg),
        )

    def slide_local_y_per_mm(self) -> float:
        """返回 Slide 每移动 1 mm 对机械臂局部 y 的扣除系数。"""

        return 1.0


def rotation_output_yaw_deg(
    shoulder_deg: float,
    elbow_deg: float,
    rotation_deg: float,
) -> float:
    """返回 Rotation 输出 frame 相对平面原点的 yaw。

    FK 与 IK 共用这一逻辑角组合，避免在 Base-frame 求解器中维护第二套
    Rotation 安装约定。
    """

    return sum(
        _require_finite(name, value)
        for name, value in (
            ("shoulder_deg", shoulder_deg),
            ("elbow_deg", elbow_deg),
            ("rotation_deg", rotation_deg),
        )
    )


def rotation_deg_for_output_yaw(
    output_yaw_deg: float,
    shoulder_deg: float,
    elbow_deg: float,
) -> float:
    """按与 FK 相同的安装约定反解未归一化 Rotation 逻辑角。"""

    output = _require_finite("output_yaw_deg", output_yaw_deg)
    shoulder = _require_finite("shoulder_deg", shoulder_deg)
    elbow = _require_finite("elbow_deg", elbow_deg)
    return output - shoulder - elbow


def load_five_axis_geometry(path: Path) -> FiveAxisGeometry:
    """加载经过操作者明确确认的 schema 1 本机几何。"""

    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = json.load(stream)
    except json.JSONDecodeError as exc:
        raise FiveAxisGeometryError(f"invalid JSON in {path}: {exc}") from exc
    except OSError as exc:
        raise FiveAxisGeometryError(
            f"cannot read current robot geometry {path}: {exc}; expected "
            "host/config/robot_geometry.json"
        ) from exc
    if not isinstance(root, dict):
        raise FiveAxisGeometryError("five-axis geometry document must be an object")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise FiveAxisGeometryError(
            f"schema_version must be {SCHEMA_VERSION}, got "
            f"{root.get('schema_version')!r}"
        )
    if root.get("geometry_confirmed") is not True:
        raise FiveAxisGeometryError(
            "geometry_confirmed must be true after link lengths and "
            "tcp_height_at_z_zero_mm have been measured and reviewed"
        )
    link_lengths = _numeric_vector(root.get("link_lengths_mm"), "link_lengths_mm", 2)
    return FiveAxisGeometry(
        link1_length_mm=link_lengths[0],
        link2_length_mm=link_lengths[1],
        tcp_height_at_z_zero_mm=_require_finite(
            "tcp_height_at_z_zero_mm",
            root.get("tcp_height_at_z_zero_mm"),
        ),
    )


def load_robot_five_axis_kinematics() -> FiveAxisKinematics:
    """默认 FK provider：加载当前机械臂正式几何。"""

    return FiveAxisKinematics(load_five_axis_geometry(DEFAULT_ROBOT_GEOMETRY_PATH))


def _numeric_vector(
    value: object,
    name: str,
    expected_length: int,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise FiveAxisGeometryError(
            f"{name} must be an array of {expected_length} finite numbers"
        )
    if len(value) != expected_length:
        raise FiveAxisGeometryError(
            f"{name} must contain exactly {expected_length} numbers"
        )
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise FiveAxisGeometryError(f"{name}[{index}] must be a number")
        converted = float(item)
        if not math.isfinite(converted):
            raise FiveAxisGeometryError(f"{name}[{index}] must be finite")
        result.append(converted)
    return tuple(result)


def _require_positive(name: str, value: object) -> float:
    converted = _require_finite(name, value)
    if converted <= 0:
        raise FiveAxisGeometryError(f"{name} must be finite and greater than zero")
    return converted


def _require_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FiveAxisGeometryError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise FiveAxisGeometryError(f"{name} must be finite")
    return converted


__all__ = [
    "DEFAULT_ROBOT_GEOMETRY_PATH",
    "FiveAxisGeometry",
    "FiveAxisGeometryError",
    "FiveAxisKinematics",
    "PlanarLocalTarget",
    "load_five_axis_geometry",
    "load_robot_five_axis_kinematics",
    "rotation_deg_for_output_yaw",
    "rotation_output_yaw_deg",
]
