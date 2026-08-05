"""版本 1 视觉 JSON 消息协议。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import TypeAlias

from vision.observation import Quaternion, Vector3


PROTOCOL_VERSION = 1


class VisionProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class CaptureRequest:
    request_id: str
    camera_frame: str
    timestamp: float
    protocol_version: int = PROTOCOL_VERSION
    type: str = "capture_request"

    def __post_init__(self) -> None:
        _header(self.protocol_version, self.type, "capture_request", self.request_id)
        _non_empty("camera_frame", self.camera_frame)
        _finite("timestamp", self.timestamp)


@dataclass(frozen=True)
class TargetDetection:
    request_id: str
    frame_id: str
    timestamp: float | None
    target_id: str | None
    confidence: float | None
    position_mm: Vector3
    orientation: Quaternion | None = None
    protocol_version: int = PROTOCOL_VERSION
    type: str = "target_detection"

    def __post_init__(self) -> None:
        _header(self.protocol_version, self.type, "target_detection", self.request_id)
        _non_empty("frame_id", self.frame_id)
        _optional_finite("timestamp", self.timestamp)
        if self.target_id is not None:
            _non_empty("target_id", self.target_id)
        confidence = _optional_finite("confidence", self.confidence)
        if confidence is not None and not 0.0 <= confidence <= 1.0:
            raise VisionProtocolError("confidence must be between 0 and 1")
        if not isinstance(self.position_mm, Vector3):
            raise TypeError("position_mm must be a Vector3")
        if self.position_mm.z <= 0.0:
            raise VisionProtocolError("position_mm.z depth must be positive")
        if self.orientation is not None and not isinstance(self.orientation, Quaternion):
            raise TypeError("orientation must be a Quaternion or None")


@dataclass(frozen=True)
class NoTarget:
    request_id: str
    reason: str
    protocol_version: int = PROTOCOL_VERSION
    type: str = "no_target"

    def __post_init__(self) -> None:
        _header(self.protocol_version, self.type, "no_target", self.request_id)
        _non_empty("reason", self.reason)


@dataclass(frozen=True)
class VisionError:
    request_id: str
    code: str
    message: str
    protocol_version: int = PROTOCOL_VERSION
    type: str = "error"

    def __post_init__(self) -> None:
        _header(self.protocol_version, self.type, "error", self.request_id)
        _non_empty("code", self.code)
        _non_empty("message", self.message)


VisionDetectionResult: TypeAlias = TargetDetection | NoTarget | VisionError
VisionMessage: TypeAlias = CaptureRequest | VisionDetectionResult


def encode_message(message: VisionMessage) -> bytes:
    if isinstance(message, CaptureRequest):
        payload = {"protocol_version": message.protocol_version, "type": message.type, "request_id": message.request_id, "camera_frame": message.camera_frame, "timestamp": message.timestamp}
    elif isinstance(message, TargetDetection):
        payload = {
            "protocol_version": message.protocol_version, "type": message.type,
            "request_id": message.request_id, "frame_id": message.frame_id,
            "timestamp": message.timestamp, "target_id": message.target_id,
            "confidence": message.confidence,
            "position_mm": {"x": message.position_mm.x, "y": message.position_mm.y, "z": message.position_mm.z},
            "orientation": None if message.orientation is None else {
                "x": message.orientation.x, "y": message.orientation.y,
                "z": message.orientation.z, "w": message.orientation.w,
            },
        }
    elif isinstance(message, NoTarget):
        payload = {"protocol_version": message.protocol_version, "type": message.type, "request_id": message.request_id, "reason": message.reason}
    elif isinstance(message, VisionError):
        payload = {"protocol_version": message.protocol_version, "type": message.type, "request_id": message.request_id, "code": message.code, "message": message.message}
    else:
        raise TypeError("unsupported vision message")
    return json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8") + b"\n"


def decode_message(payload: bytes | str) -> VisionMessage:
    if isinstance(payload, bytes):
        try:
            payload = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise VisionProtocolError("message is not valid UTF-8") from exc
    if not isinstance(payload, str):
        raise TypeError("payload must be bytes or str")
    try:
        root = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise VisionProtocolError(f"malformed JSON: {exc.msg}") from exc
    if not isinstance(root, dict):
        raise VisionProtocolError("message must be a JSON object")
    version = root.get("protocol_version")
    if version != PROTOCOL_VERSION:
        raise VisionProtocolError(f"protocol_version must be {PROTOCOL_VERSION}")
    kind = root.get("type")
    try:
        if kind == "capture_request":
            _keys(root, {"protocol_version", "type", "request_id", "camera_frame", "timestamp"})
            return CaptureRequest(root.get("request_id"), root.get("camera_frame"), root.get("timestamp"))
        if kind == "target_detection":
            _keys(root, {"protocol_version", "type", "request_id", "frame_id", "timestamp", "target_id", "confidence", "position_mm", "orientation"})
            position = _object(root.get("position_mm"), "position_mm", {"x", "y", "z"})
            orientation_value = root.get("orientation")
            orientation = None
            if orientation_value is not None:
                item = _object(orientation_value, "orientation", {"x", "y", "z", "w"})
                orientation = Quaternion(item.get("x"), item.get("y"), item.get("z"), item.get("w"))
            return TargetDetection(
                request_id=root.get("request_id"), frame_id=root.get("frame_id"),
                timestamp=root.get("timestamp"), target_id=root.get("target_id"),
                confidence=root.get("confidence"),
                position_mm=Vector3(position.get("x"), position.get("y"), position.get("z")),
                orientation=orientation,
            )
        if kind == "no_target":
            _keys(root, {"protocol_version", "type", "request_id", "reason"})
            return NoTarget(root.get("request_id"), root.get("reason"))
        if kind == "error":
            _keys(root, {"protocol_version", "type", "request_id", "code", "message"})
            return VisionError(root.get("request_id"), root.get("code"), root.get("message"))
    except (TypeError, ValueError) as exc:
        if isinstance(exc, VisionProtocolError):
            raise
        raise VisionProtocolError(str(exc)) from exc
    raise VisionProtocolError(f"unsupported message type: {kind!r}")


def _header(version: object, actual_type: object, expected_type: str, request_id: object) -> None:
    if version != PROTOCOL_VERSION:
        raise VisionProtocolError(f"protocol_version must be {PROTOCOL_VERSION}")
    if actual_type != expected_type:
        raise VisionProtocolError(f"type must be {expected_type!r}")
    _non_empty("request_id", request_id)


def _keys(root: dict[object, object], expected: set[str]) -> None:
    actual = set(root)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise VisionProtocolError(f"invalid fields: missing={missing}, extra={extra}")


def _object(value: object, name: str, expected: set[str]) -> dict[object, object]:
    if not isinstance(value, dict):
        raise VisionProtocolError(f"{name} must be an object")
    _keys(value, expected)
    return value


def _non_empty(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise VisionProtocolError(f"{name} must be a non-empty string")
    return value


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise VisionProtocolError(f"{name} must be finite")
    return float(value)


def _optional_finite(name: str, value: object) -> float | None:
    return None if value is None else _finite(name, value)


__all__ = [
    "CaptureRequest", "NoTarget", "PROTOCOL_VERSION", "TargetDetection", "VisionDetectionResult",
    "VisionError", "VisionMessage", "VisionProtocolError", "decode_message", "encode_message",
]
