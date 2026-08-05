"""用户显式指定路径时写入 JSON Lines 运行记录。"""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import json
from pathlib import Path
import subprocess
from typing import Protocol, runtime_checkable


@runtime_checkable
class ExecutionRecorder(Protocol):
    def record(self, operation: str, **fields: object) -> None: ...


class NullExecutionRecorder:
    def record(self, operation: str, **fields: object) -> None:
        return None


class JsonLinesExecutionRecorder:
    def __init__(self, path: Path, *, repository_root: Path | None = None) -> None:
        if not isinstance(path, Path):
            raise TypeError("path must be pathlib.Path")
        self.path = path
        self.repository_root = repository_root
        self.root_commit = _revision(repository_root) if repository_root else None
        submodule = repository_root / "firmware" / "stm32_motion_controller" if repository_root else None
        self.submodule_commit = _revision(submodule) if submodule and submodule.exists() else None

    def record(self, operation: str, **fields: object) -> None:
        if not isinstance(operation, str) or not operation.strip():
            raise ValueError("operation must be a non-empty string")
        payload = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "operation": operation,
            "root_commit": self.root_commit,
            "submodule_commit": self.submodule_commit,
            **fields,
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, allow_nan=False, default=_json_value, separators=(",", ":")) + "\n")


def _revision(path: Path) -> str | None:
    try:
        result = subprocess.run(
            ("git", "-C", str(path), "rev-parse", "HEAD"),
            check=True, capture_output=True, text=True, timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def _json_value(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    if hasattr(value, "translation_mm") and hasattr(value, "rpy_deg"):
        return {
            "translation_mm": [float(item) for item in value.translation_mm],
            "rotation_rpy_deg": [float(item) for item in value.rpy_deg],
        }
    if is_dataclass(value):
        # camera_T_target 是 position/orientation 的兼容派生值；不重复记录。
        excluded = {"tool_T_camera", "camera_T_target"}
        return {
            item.name: getattr(value, item.name)
            for item in fields(value)
            if item.name not in excluded
        }
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


__all__ = ["ExecutionRecorder", "JsonLinesExecutionRecorder", "NullExecutionRecorder"]
