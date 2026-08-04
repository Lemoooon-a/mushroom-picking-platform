"""Slide-zero 根下的参数化五轴正运动学。

链路为：线性 Slide/Z 平移 -> 平面 Shoulder/Elbow 2R -> Rotation yaw -> Tool/TCP。
真实连杆长度、线性轴方向、平面原点安装和 Rotation-output-to-Tool 固定变换均由
本机 JSON 显式提供；模块没有机械尺寸默认值，也不读取 startup position 或硬件。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from kinematics.planar_2r import Planar2RKinematics


SCHEMA_VERSION = 1
DEFAULT_LOCAL_GEOMETRY_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "five_axis_geometry.local.json"
)


class FiveAxisGeometryError(ValueError):
    """五轴几何缺失、未经确认或违反模型约束。"""


@dataclass(frozen=True)
class FiveAxisGeometry:
    """完整 FK 所需的纯机械几何。

    ``slide_direction_xyz`` 和 ``z_direction_xyz`` 是在 Slide-zero 中表达的单位
    方向。``slide_zero_T_planar_origin_at_zero`` 是 Slide/Z 都为 0 时平面肩关节
    原点的安装变换。``rotation_output_T_tool`` 把 Tool/TCP 坐标转换到 Rotation
    输出坐标；它的平移会随肩、肘和 Rotation 一起旋转。
    """

    link1_length_mm: float
    link2_length_mm: float
    slide_direction_xyz: tuple[float, float, float]
    z_direction_xyz: tuple[float, float, float]
    slide_zero_T_planar_origin_at_zero: RigidTransform
    rotation_output_T_tool: RigidTransform

    def __post_init__(self) -> None:
        _require_positive("link1_length_mm", self.link1_length_mm)
        _require_positive("link2_length_mm", self.link2_length_mm)
        _validate_unit_direction("slide_direction_xyz", self.slide_direction_xyz)
        _validate_unit_direction("z_direction_xyz", self.z_direction_xyz)
        for field_name in (
            "slide_zero_T_planar_origin_at_zero",
            "rotation_output_T_tool",
        ):
            if not isinstance(getattr(self, field_name), RigidTransform):
                raise TypeError(f"{field_name} must be a RigidTransform")


class FiveAxisKinematics:
    """实现 ``SlideZeroKinematics`` Protocol 的纯计算 FK provider。"""

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
        """返回 ``slide_zero_T_tool``；输入为统一逻辑 mm/deg 状态。"""

        if not isinstance(axis_state, RobotAxisState):
            raise TypeError("axis_state must be RobotAxisState")
        shoulder_rad = math.radians(axis_state.shoulder_deg)
        elbow_rad = math.radians(axis_state.elbow_deg)
        planar_point = self.planar_2r.forward(shoulder_rad, elbow_rad)

        linear_translation = (
            np.asarray(self.geometry.slide_direction_xyz) * axis_state.slide_mm
            + np.asarray(self.geometry.z_direction_xyz) * axis_state.z_mm
        )
        slide_zero_T_linear_offset = RigidTransform.from_xyz_yaw_deg(
            x_mm=float(linear_translation[0]),
            y_mm=float(linear_translation[1]),
            z_mm=float(linear_translation[2]),
            yaw_deg=0.0,
        )
        slide_zero_T_planar_origin = (
            slide_zero_T_linear_offset
            @ self.geometry.slide_zero_T_planar_origin_at_zero
        )
        output_yaw_deg = rotation_output_yaw_deg(
            axis_state.shoulder_deg,
            axis_state.elbow_deg,
            axis_state.rotation_deg,
        )
        planar_origin_T_rotation_output = RigidTransform.from_xyz_yaw_deg(
            x_mm=planar_point.x,
            y_mm=planar_point.y,
            z_mm=0.0,
            yaw_deg=output_yaw_deg,
        )
        return (
            slide_zero_T_planar_origin
            @ planar_origin_T_rotation_output
            @ self.geometry.rotation_output_T_tool
        )


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
            f"cannot read five-axis geometry {path}: {exc}; copy "
            "host/config/five_axis_geometry.example.json to "
            "host/config/five_axis_geometry.local.json and fill measured values"
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
            "geometry_confirmed must be true after all dimensions and frame "
            "directions have been measured and reviewed"
        )
    link_lengths = _numeric_vector(root.get("link_lengths_mm"), "link_lengths_mm", 2)
    slide_direction = _numeric_vector(
        root.get("slide_direction_xyz"),
        "slide_direction_xyz",
        3,
    )
    z_direction = _numeric_vector(
        root.get("z_direction_xyz"),
        "z_direction_xyz",
        3,
    )
    return FiveAxisGeometry(
        link1_length_mm=link_lengths[0],
        link2_length_mm=link_lengths[1],
        slide_direction_xyz=(
            slide_direction[0],
            slide_direction[1],
            slide_direction[2],
        ),
        z_direction_xyz=(z_direction[0], z_direction[1], z_direction[2]),
        slide_zero_T_planar_origin_at_zero=_parse_transform(
            root.get("slide_zero_T_planar_origin_at_zero"),
            "slide_zero_T_planar_origin_at_zero",
        ),
        rotation_output_T_tool=_parse_transform(
            root.get("rotation_output_T_tool"),
            "rotation_output_T_tool",
        ),
    )


def load_local_five_axis_kinematics() -> FiveAxisKinematics:
    """默认 FK provider：加载被 Git 忽略的当前机器几何。"""

    return FiveAxisKinematics(load_five_axis_geometry(DEFAULT_LOCAL_GEOMETRY_PATH))


def _parse_transform(value: object, name: str) -> RigidTransform:
    if not isinstance(value, dict):
        raise FiveAxisGeometryError(f"{name} must be an object")
    translation = _numeric_vector(
        value.get("translation_mm"),
        f"{name}.translation_mm",
        3,
    )
    rpy = _numeric_vector(
        value.get("rotation_rpy_deg"),
        f"{name}.rotation_rpy_deg",
        3,
    )
    return RigidTransform.from_xyz_rpy_deg(
        x_mm=translation[0],
        y_mm=translation[1],
        z_mm=translation[2],
        roll_deg=rpy[0],
        pitch_deg=rpy[1],
        yaw_deg=rpy[2],
    )


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


def _validate_unit_direction(name: str, value: object) -> None:
    vector = _numeric_vector(value, name, 3)
    norm = math.sqrt(sum(component * component for component in vector))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise FiveAxisGeometryError(
            f"{name} must be a unit vector, got norm {norm:.12g}"
        )


def _require_positive(name: str, value: object) -> float:
    converted = _require_finite(name, value)
    if converted <= 0:
        raise FiveAxisGeometryError(f"{name} must be finite and greater than zero")
    return converted


def _require_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise FiveAxisGeometryError(f"{name} must be finite")
    return converted


__all__ = [
    "DEFAULT_LOCAL_GEOMETRY_PATH",
    "FiveAxisGeometry",
    "FiveAxisGeometryError",
    "FiveAxisKinematics",
    "load_five_axis_geometry",
    "load_local_five_axis_kinematics",
    "rotation_deg_for_output_yaw",
    "rotation_output_yaw_deg",
]
