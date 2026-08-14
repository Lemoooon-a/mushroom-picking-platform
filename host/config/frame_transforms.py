"""Base/Slide-zero/Tool/Camera 固定变换的 JSON 配置。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

from geometry.rigid_transform import RigidTransform


SCHEMA_VERSION = 1


class FrameTransformConfigError(ValueError):
    """固定坐标变换文件缺失字段、损坏或违反 schema。"""


@dataclass(frozen=True)
class FixedFrameTransforms:
    base_T_slide_zero: RigidTransform
    tool_T_camera: RigidTransform | None

    def __post_init__(self) -> None:
        if not isinstance(self.base_T_slide_zero, RigidTransform):
            raise TypeError("base_T_slide_zero must be a RigidTransform")
        if self.tool_T_camera is not None and not isinstance(
            self.tool_T_camera, RigidTransform
        ):
            raise TypeError("tool_T_camera must be a RigidTransform or None")


@dataclass(frozen=True)
class FrameTransformsDocument:
    transforms: FixedFrameTransforms
    metadata: Mapping[str, object]

    def __post_init__(self) -> None:
        if not isinstance(self.transforms, FixedFrameTransforms):
            raise TypeError("transforms must be FixedFrameTransforms")
        object.__setattr__(self, "metadata", _validated_metadata(self.metadata))


def load_frame_transforms(path: Path) -> FixedFrameTransforms:
    return load_frame_transforms_document(path).transforms


def load_frame_transforms_document(path: Path) -> FrameTransformsDocument:
    path = _require_path(path)
    try:
        with path.open("r", encoding="utf-8") as stream:
            root = json.load(stream)
    except json.JSONDecodeError as exc:
        raise FrameTransformConfigError(
            f"invalid JSON in frame transform file {path}: {exc}"
        ) from exc
    except OSError as exc:
        raise FrameTransformConfigError(
            f"cannot read frame transform file {path}: {exc}"
        ) from exc

    return parse_frame_transforms_document(root)


def parse_frame_transforms_document(root: object) -> FrameTransformsDocument:
    """校验统一 Runtime 配置中的 ``frame_transforms`` 区块。"""

    if not isinstance(root, dict):
        raise FrameTransformConfigError("frame transform document must be an object")
    if root.get("schema_version") != SCHEMA_VERSION:
        raise FrameTransformConfigError(
            f"schema_version must be {SCHEMA_VERSION}, got "
            f"{root.get('schema_version')!r}"
        )
    base_value = root.get("base_T_slide_zero")
    base_T_slide_zero = (
        RigidTransform.identity()
        if base_value is None
        else _parse_transform(base_value, "base_T_slide_zero")
    )
    tool_value = root.get("tool_T_camera")
    tool_T_camera = (
        None
        if tool_value is None
        else _parse_transform(tool_value, "tool_T_camera")
    )
    metadata = root.get("metadata", {})
    if not isinstance(metadata, dict):
        raise FrameTransformConfigError("metadata must be an object")
    return FrameTransformsDocument(
        transforms=FixedFrameTransforms(
            base_T_slide_zero=base_T_slide_zero,
            tool_T_camera=tool_T_camera,
        ),
        metadata=metadata,
    )


def frame_transforms_document_to_dict(
    transforms: FixedFrameTransforms,
    *,
    metadata: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """返回可安全写入统一 Runtime JSON 的 frame-transform 区块。"""

    if not isinstance(transforms, FixedFrameTransforms):
        raise TypeError("transforms must be FixedFrameTransforms")
    return {
        "schema_version": SCHEMA_VERSION,
        "base_T_slide_zero": _transform_to_json(transforms.base_T_slide_zero),
        "tool_T_camera": (
            None
            if transforms.tool_T_camera is None
            else _transform_to_json(transforms.tool_T_camera)
        ),
        "metadata": _validated_metadata(metadata or {}),
    }


def save_frame_transforms(
    path: Path,
    transforms: FixedFrameTransforms,
    *,
    metadata: Mapping[str, object] | None = None,
    overwrite: bool = False,
) -> None:
    """同目录临时文件写入后原子替换；默认拒绝覆盖已有文件。"""

    path = _require_path(path)
    if not isinstance(transforms, FixedFrameTransforms):
        raise TypeError("transforms must be FixedFrameTransforms")
    if not isinstance(overwrite, bool):
        raise TypeError("overwrite must be a bool")
    if path.exists() and not overwrite:
        raise FileExistsError(
            f"frame transform file already exists: {path}; explicit overwrite required"
        )
    document = frame_transforms_document_to_dict(
        transforms,
        metadata=metadata,
    )
    try:
        payload = json.dumps(
            document,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ) + "\n"
    except (TypeError, ValueError) as exc:
        raise FrameTransformConfigError(f"document is not JSON serializable: {exc}") from exc

    path.parent.mkdir(parents=True, exist_ok=True)
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
        if overwrite:
            os.replace(temporary_path, path)
        else:
            # 同一目录 hard link 保证“目标不存在”检查与创建为一个原子操作；
            # 不会在 exists() 与 replace() 之间误覆盖并发创建的本地标定文件。
            os.link(temporary_path, path)
            temporary_path.unlink()
    except OSError:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise


def _parse_transform(value: object, name: str) -> RigidTransform:
    if not isinstance(value, dict):
        raise FrameTransformConfigError(f"{name} must be an object")
    translation = _numeric_triplet(value.get("translation_mm"), f"{name}.translation_mm")
    rpy = _numeric_triplet(value.get("rotation_rpy_deg"), f"{name}.rotation_rpy_deg")
    try:
        return RigidTransform.from_xyz_rpy_deg(
            x_mm=translation[0],
            y_mm=translation[1],
            z_mm=translation[2],
            roll_deg=rpy[0],
            pitch_deg=rpy[1],
            yaw_deg=rpy[2],
        )
    except (TypeError, ValueError) as exc:
        raise FrameTransformConfigError(f"invalid {name}: {exc}") from exc


def _numeric_triplet(value: object, name: str) -> tuple[float, float, float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise FrameTransformConfigError(f"{name} must be an array of three numbers")
    if len(value) != 3:
        raise FrameTransformConfigError(f"{name} must contain exactly three numbers")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise FrameTransformConfigError(f"{name}[{index}] must be a number")
        converted = float(item)
        if not math.isfinite(converted):
            raise FrameTransformConfigError(f"{name}[{index}] must be finite")
        result.append(converted)
    return result[0], result[1], result[2]


def _transform_to_json(transform: RigidTransform) -> dict[str, list[float]]:
    translation = transform.translation_mm
    rpy = transform.rpy_deg
    return {
        "translation_mm": [_clean_float(value) for value in translation],
        "rotation_rpy_deg": [_clean_float(value) for value in rpy],
    }


def _clean_float(value: object) -> float:
    converted = float(value)
    return 0.0 if math.isclose(converted, 0.0, abs_tol=1e-12) else converted


def _validated_metadata(metadata: Mapping[str, object]) -> dict[str, Any]:
    if not isinstance(metadata, Mapping):
        raise TypeError("metadata must be a mapping")
    if not all(isinstance(key, str) for key in metadata):
        raise FrameTransformConfigError("metadata keys must be strings")
    try:
        # JSON round-trip both validates and detaches nested mutable values.
        return json.loads(json.dumps(dict(metadata), allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise FrameTransformConfigError(f"metadata must be JSON serializable: {exc}") from exc


def _require_path(path: Path) -> Path:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    return path


__all__ = [
    "FixedFrameTransforms",
    "FrameTransformConfigError",
    "FrameTransformsDocument",
    "SCHEMA_VERSION",
    "frame_transforms_document_to_dict",
    "load_frame_transforms",
    "load_frame_transforms_document",
    "parse_frame_transforms_document",
    "save_frame_transforms",
]
