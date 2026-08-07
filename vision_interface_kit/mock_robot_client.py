#!/usr/bin/env python3
"""Standalone acceptance client for the Vision Gateway Protocol v1."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
import uuid
from typing import Any


PROTOCOL_VERSION = 1
CAMERA_FRAME = "camera_color_optical_frame"
VALID_RESPONSE_TYPES = {"target_detection", "no_target", "error"}
MAXIMUM_MESSAGE_BYTES = 65536

RESPONSE_FIELDS = {
    "target_detection": {
        "protocol_version",
        "type",
        "request_id",
        "frame_id",
        "timestamp",
        "target_id",
        "confidence",
        "position_mm",
        "orientation",
    },
    "no_target": {"protocol_version", "type", "request_id", "reason"},
    "error": {"protocol_version", "type", "request_id", "code", "message"},
}


class ProtocolError(ValueError):
    """The Vision response does not satisfy protocol v1."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one capture_request and validate one Vision response."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser


def build_request() -> dict[str, object]:
    return {
        "protocol_version": PROTOCOL_VERSION,
        "type": "capture_request",
        "request_id": f"capture-{uuid.uuid4().hex[:12]}",
        "camera_frame": CAMERA_FRAME,
        "timestamp": time.time(),
    }


def encode_line(message: dict[str, object]) -> bytes:
    return (
        json.dumps(
            message,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        + b"\n"
    )


def receive_line(connection: socket.socket) -> bytes:
    data = bytearray()
    while True:
        chunk = connection.recv(
            min(4096, MAXIMUM_MESSAGE_BYTES + 1 - len(data))
        )
        if not chunk:
            raise ProtocolError("Vision Server disconnected before newline")
        data.extend(chunk)
        if len(data) > MAXIMUM_MESSAGE_BYTES:
            raise ProtocolError(
                f"response exceeds {MAXIMUM_MESSAGE_BYTES} bytes"
            )
        newline = data.find(b"\n")
        if newline >= 0:
            if newline != len(data) - 1:
                raise ProtocolError("response contains trailing bytes after one message")
            return bytes(data[:newline])


def decode_json(payload: bytes) -> dict[str, Any]:
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError("response is not valid UTF-8") from exc
    try:
        message = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"response is not valid JSON: {exc.msg}") from exc
    if not isinstance(message, dict):
        raise ProtocolError("response must be a JSON object")
    return message


def validate_response(message: dict[str, Any], request_id: str) -> None:
    if message.get("protocol_version") != PROTOCOL_VERSION:
        raise ProtocolError(f"protocol_version must be {PROTOCOL_VERSION}")

    message_type = message.get("type")
    if message_type not in VALID_RESPONSE_TYPES:
        raise ProtocolError(
            "type must be target_detection, no_target, or error"
        )
    if message.get("request_id") != request_id:
        raise ProtocolError(
            f"response request_id={message.get('request_id')!r} does not match "
            f"request {request_id!r}"
        )

    expected = RESPONSE_FIELDS[message_type]
    actual = set(message)
    if actual != expected:
        raise ProtocolError(
            f"invalid fields: missing={sorted(expected - actual)}, "
            f"extra={sorted(actual - expected)}"
        )

    if message_type == "target_detection":
        _validate_target_detection(message)
    elif message_type == "no_target":
        _require_non_empty("reason", message["reason"])
    else:
        _require_non_empty("code", message["code"])
        _require_non_empty("message", message["message"])


def _validate_target_detection(message: dict[str, Any]) -> None:
    if message["frame_id"] != CAMERA_FRAME:
        raise ProtocolError(f"frame_id must be {CAMERA_FRAME!r}")

    position = message["position_mm"]
    if not isinstance(position, dict) or set(position) != {"x", "y", "z"}:
        raise ProtocolError("position_mm must contain exactly x, y, and z")
    for name in ("x", "y", "z"):
        _require_finite(f"position_mm.{name}", position[name])
    if position["z"] <= 0:
        raise ProtocolError("position_mm.z depth must be positive")

    timestamp = message["timestamp"]
    if timestamp is not None:
        _require_finite("timestamp", timestamp)

    target_id = message["target_id"]
    if target_id is not None:
        _require_non_empty("target_id", target_id)

    confidence = message["confidence"]
    if confidence is not None:
        value = _require_finite("confidence", confidence)
        if not 0.0 <= value <= 1.0:
            raise ProtocolError("confidence must be between 0 and 1")

    orientation = message["orientation"]
    if orientation is not None:
        if not isinstance(orientation, dict) or set(orientation) != {
            "x",
            "y",
            "z",
            "w",
        }:
            raise ProtocolError("orientation must contain exactly x, y, z, and w")
        values = [
            _require_finite(f"orientation.{name}", orientation[name])
            for name in ("x", "y", "z", "w")
        ]
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ProtocolError("orientation quaternion must have unit norm")


def _require_non_empty(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ProtocolError(f"{name} must be a non-empty string")
    return value


def _require_finite(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ProtocolError(f"{name} must be finite")
    return float(value)


def run(host: str, port: int, timeout: float) -> dict[str, Any]:
    if not host.strip():
        raise ValueError("host must be non-empty")
    if not 1 <= port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be positive")

    request = build_request()
    with socket.create_connection((host, port), timeout=timeout) as connection:
        connection.settimeout(timeout)
        connection.sendall(encode_line(request))
        response = decode_json(receive_line(connection))
    validate_response(response, request["request_id"])
    return response


def main() -> int:
    args = build_parser().parse_args()
    try:
        response = run(args.host, args.port, args.timeout)
    except (OSError, ProtocolError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(response, ensure_ascii=False, indent=2, sort_keys=True))
    print("PASS: Vision Server response conforms to protocol v1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
