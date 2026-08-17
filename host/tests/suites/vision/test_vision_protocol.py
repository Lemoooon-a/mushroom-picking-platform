from __future__ import annotations

import json
import unittest

from vision.observation import Quaternion, Vector3
from vision.protocol import (
    CaptureRequest, NoTarget, TargetDetection, VisionError, VisionProtocolError,
    decode_message, encode_message,
)
from vision.target_size import TargetSizeClass


class VisionProtocolTests(unittest.TestCase):
    def test_request_encode_round_trip(self) -> None:
        request = CaptureRequest("capture-1", "camera_optical", 12.5)
        self.assertEqual(decode_message(encode_message(request)), request)

    def test_detection_decode_with_optional_orientation(self) -> None:
        detection = TargetDetection(
            "capture-1", "camera_optical", 12.5, "mushroom-1", 0.95,
            Vector3(1, 2, 3), Quaternion(0, 0, 0, 1),
        )
        self.assertEqual(decode_message(encode_message(detection)), detection)
        without_orientation = json.loads(encode_message(detection))
        without_orientation["orientation"] = None
        self.assertIsNone(decode_message(json.dumps(without_orientation)).orientation)

        all_optional_null = TargetDetection(
            "capture-2",
            "camera_color_optical_frame",
            None,
            None,
            None,
            Vector3(1, 2, 3),
            None,
        )
        self.assertEqual(
            decode_message(encode_message(all_optional_null)),
            all_optional_null,
        )

    def test_detection_size_class_is_compatible_and_strict(self) -> None:
        oversized = TargetDetection(
            "capture-1",
            "camera_optical",
            12.5,
            "mushroom-1",
            0.95,
            Vector3(1, 2, 3),
            size_class=TargetSizeClass.OVERSIZED,
        )
        self.assertEqual(
            decode_message(encode_message(oversized)).size_class,
            TargetSizeClass.OVERSIZED,
        )

        legacy = json.loads(encode_message(oversized))
        legacy.pop("size_class")
        self.assertEqual(
            decode_message(json.dumps(legacy)).size_class,
            TargetSizeClass.NORMAL,
        )

        for invalid in (None, "large", 1):
            with self.subTest(invalid=invalid):
                payload = dict(legacy, size_class=invalid)
                with self.assertRaisesRegex(VisionProtocolError, "size_class"):
                    decode_message(json.dumps(payload))

    def test_no_target_and_error(self) -> None:
        self.assertEqual(decode_message(encode_message(NoTarget("capture-1", "no_detection"))), NoTarget("capture-1", "no_detection"))
        error = VisionError("capture-1", "INVALID_DEPTH", "bad depth")
        self.assertEqual(decode_message(encode_message(error)), error)

    def test_malformed_json_and_invalid_schema_are_rejected(self) -> None:
        with self.assertRaisesRegex(VisionProtocolError, "malformed JSON"):
            decode_message("{")
        with self.assertRaisesRegex(VisionProtocolError, "invalid fields"):
            decode_message('{"protocol_version":1,"type":"no_target","request_id":"x","reason":"none","extra":1}')
        with self.assertRaisesRegex(VisionProtocolError, "depth must be positive"):
            TargetDetection("x", "camera", 0, None, None, Vector3(0, 0, 0))


if __name__ == "__main__":
    unittest.main()
