from __future__ import annotations

import socket
import unittest
from unittest.mock import patch

from vision.gateway import (
    FakeVisionGateway, JsonSocketVisionGateway, VisionGatewayTimeout,
    VisionMessageTooLarge, VisionRequestMismatch,
)
from vision.observation import Vector3
from vision.protocol import CaptureRequest, NoTarget, TargetDetection, encode_message


class _Socket:
    def __init__(self, payload: bytes | BaseException) -> None:
        self.payload = payload

    def __enter__(self): return self
    def __exit__(self, *_args): return False
    def settimeout(self, _timeout): pass
    def sendall(self, payload): self.sent = payload
    def recv(self, _size):
        if isinstance(self.payload, BaseException):
            raise self.payload
        payload, self.payload = self.payload, b""
        return payload


class VisionGatewayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.request = CaptureRequest("capture-1", "camera_optical", 1.0)

    def test_fake_gateway_and_request_mismatch(self) -> None:
        result = NoTarget("capture-1", "no_detection")
        gateway = FakeVisionGateway([result])
        self.assertIs(gateway.request_target(self.request, 1), result)
        with self.assertRaises(VisionRequestMismatch):
            FakeVisionGateway([NoTarget("other", "none")]).request_target(self.request, 1)

    def test_socket_detection_and_request_mismatch(self) -> None:
        detection = TargetDetection("capture-1", "camera_optical", 1, None, 0.9, Vector3(1, 2, 3))
        fake = _Socket(encode_message(detection))
        with patch("vision.gateway.socket.create_connection", return_value=fake):
            result = JsonSocketVisionGateway("127.0.0.1", 9000).request_target(self.request, 1)
        self.assertEqual(result, detection)
        mismatch = _Socket(encode_message(NoTarget("other", "none")))
        with patch("vision.gateway.socket.create_connection", return_value=mismatch), self.assertRaises(VisionRequestMismatch):
            JsonSocketVisionGateway("127.0.0.1", 9000).request_target(self.request, 1)

    def test_timeout_and_oversized_response(self) -> None:
        with patch("vision.gateway.socket.create_connection", side_effect=socket.timeout()), self.assertRaises(VisionGatewayTimeout):
            JsonSocketVisionGateway("127.0.0.1", 9000).request_target(self.request, 0.1)
        fake = _Socket(b"x" * 65)
        with patch("vision.gateway.socket.create_connection", return_value=fake), self.assertRaises(VisionMessageTooLarge):
            JsonSocketVisionGateway("127.0.0.1", 9000, maximum_message_bytes=64).request_target(self.request, 1)


if __name__ == "__main__":
    unittest.main()
