"""视觉进程通信策略；默认不代表真实视觉已验证。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


class VisionRuntimeConfigError(ValueError):
    pass


@dataclass(frozen=True)
class VisionRuntimeConfig:
    validated: bool = False
    camera_frame: str = "camera_optical"
    host: str | None = None
    port: int | None = None
    timeout_s: float = 2.0
    maximum_message_bytes: int = 65536
    minimum_confidence: float = 0.0
    maximum_observation_age_s: float = 2.0

    def __post_init__(self) -> None:
        if not isinstance(self.validated, bool):
            raise TypeError("validated must be a bool")
        if not isinstance(self.camera_frame, str) or not self.camera_frame.strip():
            raise ValueError("camera_frame must be a non-empty string")
        if self.host is not None and (not isinstance(self.host, str) or not self.host.strip()):
            raise ValueError("host must be a non-empty string or None")
        if self.port is not None and (isinstance(self.port, bool) or not isinstance(self.port, int) or not 1 <= self.port <= 65535):
            raise ValueError("port must be between 1 and 65535 or None")
        if self.validated and (self.host is None or self.port is None):
            raise ValueError("validated socket vision requires host and port")
        _positive("timeout_s", self.timeout_s)
        if isinstance(self.maximum_message_bytes, bool) or not isinstance(self.maximum_message_bytes, int) or self.maximum_message_bytes < 64:
            raise ValueError("maximum_message_bytes must be >= 64")
        confidence = _finite("minimum_confidence", self.minimum_confidence)
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_confidence must be between 0 and 1")
        _positive("maximum_observation_age_s", self.maximum_observation_age_s)


def load_vision_runtime_config(path: Path) -> VisionRuntimeConfig:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VisionRuntimeConfigError(f"cannot load vision runtime config {path}: {exc}") from exc
    return parse_vision_runtime_config(root)


def parse_vision_runtime_config(root: object) -> VisionRuntimeConfig:
    """校验统一 Runtime 配置中的 ``vision_runtime`` 区块。"""

    if not isinstance(root, dict) or root.get("schema_version") != 1:
        raise VisionRuntimeConfigError("vision runtime schema_version must be 1")
    try:
        return VisionRuntimeConfig(**{key: value for key, value in root.items() if key != "schema_version"})
    except (TypeError, ValueError) as exc:
        raise VisionRuntimeConfigError(str(exc)) from exc


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _positive(name: str, value: object) -> float:
    converted = _finite(name, value)
    if converted <= 0.0:
        raise ValueError(f"{name} must be positive")
    return converted


DEFAULT_VISION_RUNTIME_CONFIG = VisionRuntimeConfig()


__all__ = [
    "DEFAULT_VISION_RUNTIME_CONFIG",
    "VisionRuntimeConfig",
    "VisionRuntimeConfigError",
    "load_vision_runtime_config",
    "parse_vision_runtime_config",
]
