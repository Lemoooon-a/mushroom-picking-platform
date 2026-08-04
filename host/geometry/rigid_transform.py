"""经过严格验证的三维刚体变换。

``A_T_B`` 表示把 B 坐标系中的量转换到 A 坐标系。组合顺序为
``A_T_C = A_T_B @ B_T_C``。RPY 使用固定轴 x/y/z 的 roll、pitch、yaw，
其旋转矩阵为 ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence

import numpy as np


_MATRIX_ATOL = 1e-9
_GIMBAL_LOCK_ATOL = 1e-12


def normalize_angle_deg(angle_deg: float) -> float:
    """把有限角度归一化到 ``[-180, 180)``。"""

    value = _finite_real("angle_deg", angle_deg)
    wrapped = (value + 180.0) % 360.0 - 180.0
    return 0.0 if math.isclose(wrapped, 0.0, abs_tol=1e-12) else wrapped


def angular_difference_deg(actual_deg: float, expected_deg: float) -> float:
    """返回 ``actual - expected`` 的最短有符号角差。"""

    return normalize_angle_deg(
        _finite_real("actual_deg", actual_deg)
        - _finite_real("expected_deg", expected_deg)
    )


@dataclass(frozen=True, eq=False)
class RigidTransform:
    """不可变的齐次三维刚体变换，平移单位由调用方统一为 mm。"""

    matrix: np.ndarray

    def __post_init__(self) -> None:
        try:
            matrix = np.asarray(self.matrix, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("matrix must contain real numeric values") from exc
        if matrix.shape != (4, 4):
            raise ValueError(f"matrix must have shape (4, 4), got {matrix.shape}")
        if not np.isfinite(matrix).all():
            raise ValueError("matrix must contain only finite values")
        if not np.allclose(
            matrix[3],
            np.array((0.0, 0.0, 0.0, 1.0)),
            rtol=0.0,
            atol=_MATRIX_ATOL,
        ):
            raise ValueError("matrix last row must be [0, 0, 0, 1]")

        rotation = matrix[:3, :3]
        if not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            rtol=0.0,
            atol=_MATRIX_ATOL,
        ):
            raise ValueError("matrix rotation must be orthogonal")
        determinant = float(np.linalg.det(rotation))
        if not math.isclose(
            determinant,
            1.0,
            rel_tol=0.0,
            abs_tol=_MATRIX_ATOL,
        ):
            raise ValueError(
                "matrix rotation determinant must be +1; reflections are invalid"
            )

        owned = np.array(matrix, dtype=float, copy=True)
        owned.setflags(write=False)
        object.__setattr__(self, "matrix", owned)

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(np.eye(4, dtype=float))

    @classmethod
    def from_xyz_rpy_deg(
        cls,
        *,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> "RigidTransform":
        """由平移和固定轴 RPY 构造变换。

        应用到点时先 roll、再 pitch、最后 yaw，即
        ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``。
        """

        x = _finite_real("x_mm", x_mm)
        y = _finite_real("y_mm", y_mm)
        z = _finite_real("z_mm", z_mm)
        roll = math.radians(_finite_real("roll_deg", roll_deg))
        pitch = math.radians(_finite_real("pitch_deg", pitch_deg))
        yaw = math.radians(_finite_real("yaw_deg", yaw_deg))

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation_x = np.array(
            ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
        )
        rotation_y = np.array(
            ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
        )
        rotation_z = np.array(
            ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
        )
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = rotation_z @ rotation_y @ rotation_x
        matrix[:3, 3] = (x, y, z)
        return cls(matrix)

    @classmethod
    def from_xyz_yaw_deg(
        cls,
        *,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float,
    ) -> "RigidTransform":
        return cls.from_xyz_rpy_deg(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=yaw_deg,
        )

    def inverse(self) -> "RigidTransform":
        rotation = self.matrix[:3, :3]
        translation = self.matrix[:3, 3]
        inverse_matrix = np.eye(4, dtype=float)
        inverse_matrix[:3, :3] = rotation.T
        inverse_matrix[:3, 3] = -(rotation.T @ translation)
        return RigidTransform(inverse_matrix)

    def compose(self, other: "RigidTransform") -> "RigidTransform":
        """返回 ``self @ other``，例如 ``A_T_B.compose(B_T_C)``。"""

        if not isinstance(other, RigidTransform):
            raise TypeError("other must be a RigidTransform")
        return RigidTransform(self.matrix @ other.matrix)

    def __matmul__(self, other: object) -> "RigidTransform":
        if not isinstance(other, RigidTransform):
            return NotImplemented
        return self.compose(other)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, RigidTransform) and bool(
            np.array_equal(self.matrix, other.matrix)
        )

    def __hash__(self) -> int:
        return hash(self.matrix.tobytes())

    def transform_point(self, point_xyz_mm: Sequence[float]) -> np.ndarray:
        try:
            point = np.asarray(point_xyz_mm, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("point_xyz_mm must contain real numeric values") from exc
        if point.shape != (3,):
            raise ValueError(f"point_xyz_mm must have shape (3,), got {point.shape}")
        if not np.isfinite(point).all():
            raise ValueError("point_xyz_mm must contain only finite values")
        return self.matrix[:3, :3] @ point + self.matrix[:3, 3]

    @property
    def translation_mm(self) -> np.ndarray:
        return np.array(self.matrix[:3, 3], copy=True)

    @property
    def rotation_matrix(self) -> np.ndarray:
        return np.array(self.matrix[:3, :3], copy=True)

    @property
    def rpy_deg(self) -> np.ndarray:
        """按构造约定返回一个确定的 ``[roll, pitch, yaw]`` 解。"""

        rotation = self.matrix[:3, :3]
        cos_pitch = math.hypot(float(rotation[0, 0]), float(rotation[1, 0]))
        pitch = math.atan2(-float(rotation[2, 0]), cos_pitch)
        if cos_pitch > _GIMBAL_LOCK_ATOL:
            roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
            yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        else:
            # Gimbal lock 时 roll/yaw 不可分别识别；固定 yaw=0，保留等效旋转。
            roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
            yaw = 0.0
        return np.degrees(np.array((roll, pitch, yaw), dtype=float))

    @property
    def yaw_deg(self) -> float:
        return normalize_angle_deg(float(self.rpy_deg[2]))


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise TypeError(f"{name} must be a finite real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


__all__ = [
    "RigidTransform",
    "angular_difference_deg",
    "normalize_angle_deg",
]
