"""当前机械臂唯一的业务 Runtime JSON 配置。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Mapping, cast

from application.grasp_profile import GraspProfile
from application.scan_pick import ScanPickProfile
from config.frame_transforms import (
    FixedFrameTransforms,
    FrameTransformsDocument,
    frame_transforms_document_to_dict,
    parse_frame_transforms_document,
)
from config.project.grasp_strategy import parse_validated_grasp_profile
from config.project.scan_pick import parse_validated_scan_pick_profile
from config.project.vision_runtime import (
    VisionRuntimeConfig,
    parse_vision_runtime_config,
)
from config.tray_workspace import (
    TrayWorkspaceConfig,
    parse_tray_workspace_config,
)


SCHEMA_VERSION = 1
DEFAULT_ROBOT_RUNTIME_PATH = Path(__file__).with_name("robot_runtime.json")
HOST_ROOT = DEFAULT_ROBOT_RUNTIME_PATH.parent.parent
RUNTIME_CACHE_ROOT = HOST_ROOT / "runtime"
_REQUIRED_SECTIONS = (
    "frame_transforms",
    "tray_workspace",
    "vision_runtime",
    "grasp_profile",
    "scan_pick",
    "recording",
)


class RobotRuntimeConfigError(ValueError):
    """统一 Runtime 文件缺失、损坏或任一区块无效。"""


@dataclass(frozen=True)
class RecordingConfig:
    enabled: bool
    jsonl_path: Path

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise TypeError("recording.enabled must be a bool")
        if not isinstance(self.jsonl_path, Path):
            raise TypeError("recording.jsonl_path must be pathlib.Path")
        if not self.jsonl_path.is_absolute():
            raise ValueError("recording.jsonl_path must be an absolute path")


@dataclass(frozen=True)
class RobotRuntimeConfig:
    frame_transforms: FrameTransformsDocument
    tray_workspace: TrayWorkspaceConfig
    vision_runtime: VisionRuntimeConfig
    grasp_profile: GraspProfile
    scan_pick: ScanPickProfile
    recording: RecordingConfig
    source_path: Path

    def __post_init__(self) -> None:
        if not isinstance(self.frame_transforms, FrameTransformsDocument):
            raise TypeError("frame_transforms must be FrameTransformsDocument")
        if not isinstance(self.tray_workspace, TrayWorkspaceConfig):
            raise TypeError("tray_workspace must be TrayWorkspaceConfig")
        if not isinstance(self.vision_runtime, VisionRuntimeConfig):
            raise TypeError("vision_runtime must be VisionRuntimeConfig")
        if not isinstance(self.grasp_profile, GraspProfile):
            raise TypeError("grasp_profile must be GraspProfile")
        if not isinstance(self.scan_pick, ScanPickProfile):
            raise TypeError("scan_pick must be ScanPickProfile")
        if not isinstance(self.recording, RecordingConfig):
            raise TypeError("recording must be RecordingConfig")
        if not isinstance(self.source_path, Path):
            raise TypeError("source_path must be pathlib.Path")


def load_robot_runtime_config(
    path: Path = DEFAULT_ROBOT_RUNTIME_PATH,
) -> RobotRuntimeConfig:
    """一次读取并完整校验当前机械臂的业务 Runtime 配置。"""

    checked = _require_path(path)
    return parse_robot_runtime_config(_read_document(checked), source_path=checked)


def parse_robot_runtime_config(
    root: object,
    *,
    source_path: Path = DEFAULT_ROBOT_RUNTIME_PATH,
) -> RobotRuntimeConfig:
    """从已解析 JSON 构造强类型 Runtime 配置。"""

    source_path = _require_path(source_path)
    if not isinstance(root, dict):
        raise RobotRuntimeConfigError("robot runtime document must be an object")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise RobotRuntimeConfigError(
            f"schema_version must be {SCHEMA_VERSION}, got "
            f"{root.get('schema_version')!r}"
        )
    missing = tuple(section for section in _REQUIRED_SECTIONS if section not in root)
    if missing:
        raise RobotRuntimeConfigError(
            "missing robot runtime sections: " + ", ".join(missing)
        )

    parsed: dict[str, object] = {}
    parsers = {
        "frame_transforms": parse_frame_transforms_document,
        "tray_workspace": parse_tray_workspace_config,
        "vision_runtime": parse_vision_runtime_config,
        "grasp_profile": parse_validated_grasp_profile,
        "scan_pick": parse_validated_scan_pick_profile,
        "recording": _parse_recording_config,
    }
    for section, parser in parsers.items():
        try:
            parsed[section] = parser(root[section])
        except (TypeError, ValueError) as exc:
            raise RobotRuntimeConfigError(f"{section}: {exc}") from exc

    return RobotRuntimeConfig(
        frame_transforms=cast(FrameTransformsDocument, parsed["frame_transforms"]),
        tray_workspace=cast(TrayWorkspaceConfig, parsed["tray_workspace"]),
        vision_runtime=cast(VisionRuntimeConfig, parsed["vision_runtime"]),
        grasp_profile=cast(GraspProfile, parsed["grasp_profile"]),
        scan_pick=cast(ScanPickProfile, parsed["scan_pick"]),
        recording=cast(RecordingConfig, parsed["recording"]),
        source_path=source_path,
    )


def update_robot_runtime_frame_transforms(
    path: Path,
    transforms: FixedFrameTransforms,
    *,
    metadata: Mapping[str, object] | None = None,
) -> None:
    """原子更新唯一 Runtime 文件的 frame-transforms 区块。"""

    checked = _require_path(path)
    root = _read_document(checked)
    parse_robot_runtime_config(root, source_path=checked)
    root["frame_transforms"] = frame_transforms_document_to_dict(
        transforms,
        metadata=metadata,
    )
    parse_robot_runtime_config(root, source_path=checked)
    _write_document_atomic(checked, root)


def _parse_recording_config(value: object) -> RecordingConfig:
    if not isinstance(value, dict):
        raise RobotRuntimeConfigError("recording must be an object")
    if set(value) != {"enabled", "jsonl_path"}:
        raise RobotRuntimeConfigError(
            "recording must contain exactly enabled and jsonl_path"
        )
    path = value.get("jsonl_path")
    if not isinstance(path, str) or not path.strip():
        raise RobotRuntimeConfigError(
            "recording.jsonl_path must be a non-empty string"
        )
    configured_path = Path(path)
    if configured_path.is_absolute():
        raise RobotRuntimeConfigError(
            "recording.jsonl_path must be relative to the host directory"
        )
    resolved_path = (HOST_ROOT / configured_path).resolve()
    try:
        resolved_path.relative_to(RUNTIME_CACHE_ROOT.resolve())
    except ValueError as exc:
        raise RobotRuntimeConfigError(
            "recording.jsonl_path must stay inside host/runtime"
        ) from exc
    return RecordingConfig(enabled=value.get("enabled"), jsonl_path=resolved_path)


def _read_document(path: Path) -> dict[str, object]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = json.load(stream)
    except json.JSONDecodeError as exc:
        raise RobotRuntimeConfigError(
            f"invalid JSON in robot runtime config {path}: {exc}"
        ) from exc
    except OSError as exc:
        raise RobotRuntimeConfigError(
            f"cannot read robot runtime config {path}: {exc}"
        ) from exc
    if not isinstance(root, dict):
        raise RobotRuntimeConfigError("robot runtime document must be an object")
    return root


def _write_document_atomic(path: Path, root: dict[str, object]) -> None:
    try:
        payload = json.dumps(
            root,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ) + "\n"
    except (TypeError, ValueError) as exc:
        raise RobotRuntimeConfigError(
            f"robot runtime document is not JSON serializable: {exc}"
        ) from exc
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except OSError as exc:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise RobotRuntimeConfigError(
            f"cannot write robot runtime config {path}: {exc}"
        ) from exc


def _require_path(path: Path) -> Path:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    return path


__all__ = [
    "DEFAULT_ROBOT_RUNTIME_PATH",
    "RecordingConfig",
    "RobotRuntimeConfig",
    "RobotRuntimeConfigError",
    "load_robot_runtime_config",
    "parse_robot_runtime_config",
    "update_robot_runtime_frame_transforms",
]
