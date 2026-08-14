"""Base frame 中培养槽任务工作区的集中配置。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


SCHEMA_VERSION = 1


class TrayWorkspaceConfigError(ValueError):
    """培养槽工作区配置缺失、未确认或格式无效。"""


@dataclass(frozen=True)
class TrayWorkspaceConfig:
    """Base frame 中最终任务 TCP 目标允许进入的闭区间。"""

    x_min_mm: float
    x_max_mm: float
    y_min_mm: float
    y_max_mm: float
    z_min_mm: float
    z_max_mm: float
    boundary_tolerance_mm: float = 1e-6

    def __post_init__(self) -> None:
        for field_name in (
            "x_min_mm",
            "x_max_mm",
            "y_min_mm",
            "y_max_mm",
            "z_min_mm",
            "z_max_mm",
            "boundary_tolerance_mm",
        ):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{field_name} must be a finite real number")
            converted = float(value)
            if not math.isfinite(converted):
                raise ValueError(f"{field_name} must be finite")
            object.__setattr__(self, field_name, converted)
        for axis in ("x", "y", "z"):
            minimum = getattr(self, f"{axis}_min_mm")
            maximum = getattr(self, f"{axis}_max_mm")
            if minimum > maximum:
                raise ValueError(
                    f"{axis}_min_mm must not exceed {axis}_max_mm"
                )
        if self.boundary_tolerance_mm < 0.0:
            raise ValueError("boundary_tolerance_mm must be non-negative")


def load_tray_workspace_config(path: Path) -> TrayWorkspaceConfig:
    """加载经过用户明确确认的 Base-frame 培养槽边界。"""

    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = json.load(stream)
    except json.JSONDecodeError as exc:
        raise TrayWorkspaceConfigError(
            f"invalid JSON in tray workspace file {path}: {exc}"
        ) from exc
    except OSError as exc:
        raise TrayWorkspaceConfigError(
            f"cannot read tray workspace file {path}: {exc}"
        ) from exc
    return parse_tray_workspace_config(root)


def parse_tray_workspace_config(root: object) -> TrayWorkspaceConfig:
    """校验统一 Runtime 配置中的 ``tray_workspace`` 区块。"""

    if not isinstance(root, dict):
        raise TrayWorkspaceConfigError("tray workspace document must be an object")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise TrayWorkspaceConfigError(
            f"schema_version must be {SCHEMA_VERSION}, got "
            f"{root.get('schema_version')!r}"
        )
    if root.get("frame") != "base":
        raise TrayWorkspaceConfigError("frame must be 'base'")
    metadata = root.get("metadata")
    if not isinstance(metadata, dict) or metadata.get("validated") is not True:
        raise TrayWorkspaceConfigError(
            "tray workspace boundaries are not user-validated; "
            "metadata.validated must be true"
        )
    field_names = (
        "x_min_mm",
        "x_max_mm",
        "y_min_mm",
        "y_max_mm",
        "z_min_mm",
        "z_max_mm",
    )
    missing = tuple(name for name in field_names if name not in root)
    if missing:
        raise TrayWorkspaceConfigError(
            "missing tray workspace fields: " + ", ".join(missing)
        )
    try:
        return TrayWorkspaceConfig(
            **{name: root[name] for name in field_names},
            boundary_tolerance_mm=root.get("boundary_tolerance_mm", 1e-6),
        )
    except (TypeError, ValueError) as exc:
        raise TrayWorkspaceConfigError(f"invalid tray workspace bounds: {exc}") from exc


__all__ = [
    "SCHEMA_VERSION",
    "TrayWorkspaceConfig",
    "TrayWorkspaceConfigError",
    "load_tray_workspace_config",
    "parse_tray_workspace_config",
]
