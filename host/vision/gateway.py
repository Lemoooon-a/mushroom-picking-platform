"""视觉消息传输边界；gateway 从不规划或执行机器人动作。"""

from __future__ import annotations

from collections.abc import Callable, Iterable
import socket
from typing import Protocol, runtime_checkable

from vision.protocol import (
    CaptureRequest, VisionDetectionResult, VisionProtocolError,
    decode_message, encode_message,
)


class VisionGatewayError(RuntimeError):
    pass


class VisionGatewayTimeout(VisionGatewayError):
    pass


class VisionGatewayDisconnected(VisionGatewayError):
    pass


class VisionMessageTooLarge(VisionGatewayError):
    pass


class VisionRequestMismatch(VisionGatewayError):
    pass


@runtime_checkable
class VisionGateway(Protocol):
    def request_target(self, request: CaptureRequest, timeout_s: float) -> VisionDetectionResult: ...


class FakeVisionGateway:
    """确定性的离线 gateway；每次请求消费一个结果或调用 responder。"""

    def __init__(self, responses: Iterable[VisionDetectionResult] | None = None, *, responder: Callable[[CaptureRequest], VisionDetectionResult] | None = None) -> None:
        if responses is not None and responder is not None:
            raise ValueError("provide responses or responder, not both")
        self._responses = iter(responses or ())
        self._responder = responder
        self.requests: list[CaptureRequest] = []

    def request_target(self, request: CaptureRequest, timeout_s: float) -> VisionDetectionResult:
        if not isinstance(request, CaptureRequest):
            raise TypeError("request must be a CaptureRequest")
        _timeout(timeout_s)
        self.requests.append(request)
        try:
            result = self._responder(request) if self._responder is not None else next(self._responses)
        except StopIteration as exc:
            raise VisionGatewayError("fake vision response queue is empty") from exc
        _match(request, result)
        return result


class JsonSocketVisionGateway:
    """一次连接完成一次 TCP JSON Lines request/response，不自动重连。"""

    def __init__(self, host: str, port: int, *, maximum_message_bytes: int = 65536) -> None:
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if isinstance(maximum_message_bytes, bool) or not isinstance(maximum_message_bytes, int) or maximum_message_bytes < 64:
            raise ValueError("maximum_message_bytes must be an integer >= 64")
        self.host = host
        self.port = port
        self.maximum_message_bytes = maximum_message_bytes

    def request_target(self, request: CaptureRequest, timeout_s: float) -> VisionDetectionResult:
        if not isinstance(request, CaptureRequest):
            raise TypeError("request must be a CaptureRequest")
        timeout = _timeout(timeout_s)
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as connection:
                connection.settimeout(timeout)
                connection.sendall(encode_message(request))
                payload = _read_line(connection, self.maximum_message_bytes)
        except (TimeoutError, socket.timeout) as exc:
            raise VisionGatewayTimeout("vision request timed out") from exc
        except VisionGatewayError:
            raise
        except OSError as exc:
            raise VisionGatewayDisconnected(f"vision socket failed: {exc}") from exc
        try:
            result = decode_message(payload)
        except VisionProtocolError as exc:
            raise VisionGatewayError(f"invalid vision response: {exc}") from exc
        if isinstance(result, CaptureRequest):
            raise VisionGatewayError("vision peer returned a request instead of a result")
        _match(request, result)
        return result


def _read_line(connection: socket.socket, maximum: int) -> bytes:
    data = bytearray()
    while True:
        chunk = connection.recv(min(4096, maximum + 1 - len(data)))
        if not chunk:
            raise VisionGatewayDisconnected("vision peer disconnected before newline")
        data.extend(chunk)
        if len(data) > maximum:
            raise VisionMessageTooLarge(f"vision response exceeds {maximum} bytes")
        newline = data.find(b"\n")
        if newline >= 0:
            if newline != len(data) - 1:
                raise VisionGatewayError("vision response contains trailing bytes after one message")
            return bytes(data[:newline])


def _timeout(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError("timeout_s must be positive")
    return float(value)


def _match(request: CaptureRequest, result: VisionDetectionResult) -> None:
    if result.request_id != request.request_id:
        raise VisionRequestMismatch(
            f"response request_id={result.request_id!r} does not match request {request.request_id!r}"
        )


__all__ = [
    "FakeVisionGateway", "JsonSocketVisionGateway", "VisionGateway", "VisionGatewayDisconnected",
    "VisionGatewayError", "VisionGatewayTimeout", "VisionMessageTooLarge", "VisionRequestMismatch",
]
