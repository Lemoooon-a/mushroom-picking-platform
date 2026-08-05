"""最终任务 TCP 目标的培养槽工作区门限。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from config.tray_workspace import TrayWorkspaceConfig


class TargetOutsideTrayWorkspace(ValueError):
    """目标不属于配置的 Base-frame 培养槽任务工作区。"""


@dataclass(frozen=True)
class TrayWorkspaceCheck:
    allowed: bool
    x_allowed: bool
    y_allowed: bool
    z_allowed: bool
    failed_dimensions: tuple[str, ...]


class TrayWorkspace:
    """检查最终任务目标；不修改、不裁剪任何坐标。"""

    def __init__(self, config: TrayWorkspaceConfig):
        if not isinstance(config, TrayWorkspaceConfig):
            raise TypeError("config must be TrayWorkspaceConfig")
        self._config = config

    @property
    def config(self) -> TrayWorkspaceConfig:
        return self._config

    def check_xyz(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
    ) -> TrayWorkspaceCheck:
        config = self._config
        tolerance = config.boundary_tolerance_mm
        allowed_by_dimension = {
            "x": _coordinate_allowed(
                x_mm,
                config.x_min_mm,
                config.x_max_mm,
                tolerance,
            ),
            "y": _coordinate_allowed(
                y_mm,
                config.y_min_mm,
                config.y_max_mm,
                tolerance,
            ),
            "z": _coordinate_allowed(
                z_mm,
                config.z_min_mm,
                config.z_max_mm,
                tolerance,
            ),
        }
        failed = tuple(
            dimension
            for dimension, allowed in allowed_by_dimension.items()
            if not allowed
        )
        return TrayWorkspaceCheck(
            allowed=not failed,
            x_allowed=allowed_by_dimension["x"],
            y_allowed=allowed_by_dimension["y"],
            z_allowed=allowed_by_dimension["z"],
            failed_dimensions=failed,
        )

    def require_xyz_allowed(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
    ) -> None:
        check = self.check_xyz(x_mm, y_mm, z_mm)
        if check.allowed:
            return
        config = self._config
        failed = "\n".join(f"  {name}" for name in check.failed_dimensions)
        raise TargetOutsideTrayWorkspace(
            "Target outside tray workspace.\n\n"
            "Requested:\n"
            f"  x = {_format_requested(x_mm)} mm\n"
            f"  y = {_format_requested(y_mm)} mm\n"
            f"  z = {_format_requested(z_mm)} mm\n\n"
            "Allowed:\n"
            f"  x = [{config.x_min_mm:g}, {config.x_max_mm:g}] mm\n"
            f"  y = [{config.y_min_mm:g}, {config.y_max_mm:g}] mm\n"
            f"  z = [{config.z_min_mm:g}, {config.z_max_mm:g}] mm\n\n"
            "Failed dimensions:\n"
            f"{failed}"
        )


def _coordinate_allowed(
    value: object,
    minimum: float,
    maximum: float,
    tolerance: float,
) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    converted = float(value)
    return (
        math.isfinite(converted)
        and minimum - tolerance <= converted <= maximum + tolerance
    )


def _format_requested(value: object) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return repr(value)
    return f"{float(value):g}"


__all__ = [
    "TargetOutsideTrayWorkspace",
    "TrayWorkspace",
    "TrayWorkspaceCheck",
]
