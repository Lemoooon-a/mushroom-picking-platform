"""统一运动协议类型的无硬件依赖测试。"""

from __future__ import annotations

from dataclasses import FrozenInstanceError, fields
import math
import unittest

from motion.unified_protocol import (
    ArrivalConfig,
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisTarget,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisTarget,
)


CAPABILITIES = AxisCapabilities(True, True, True, False, True, False, True)


class AxisProtocolTests(unittest.TestCase):
    def test_axis_names_are_stable(self) -> None:
        self.assertEqual(
            [axis.value for axis in AxisName],
            ["slide", "z", "shoulder", "elbow", "rotation"],
        )

    def test_linear_and_rotary_units_are_explicit(self) -> None:
        linear = AxisDescriptor(
            AxisName.SLIDE,
            "Slide",
            AxisKind.LINEAR,
            "mm",
            "mm/s",
            "mm/s²",
            0.0,
            800.0,
            CAPABILITIES,
        )
        rotary = AxisDescriptor(
            AxisName.SHOULDER,
            "Shoulder",
            AxisKind.ROTARY,
            "deg",
            "deg/s",
            "deg/s²",
            -60.0,
            70.0,
            CAPABILITIES,
        )
        self.assertEqual((linear.kind, linear.position_unit), (AxisKind.LINEAR, "mm"))
        self.assertEqual((rotary.kind, rotary.position_unit), (AxisKind.ROTARY, "deg"))

    def test_axis_target_accepts_finite_positive_profile(self) -> None:
        target = AxisTarget(AxisName.Z, 12.345, 1.25, 2.5)
        self.assertEqual(target.position, 12.345)

    def test_non_finite_position_is_rejected(self) -> None:
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                AxisTarget(AxisName.SLIDE, value)

    def test_non_positive_velocity_and_acceleration_are_rejected(self) -> None:
        for kwargs in ({"velocity": 0.0}, {"velocity": -1.0}, {"acceleration": 0.0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                AxisTarget(AxisName.SLIDE, 1.0, **kwargs)

    def test_multi_axis_target_rejects_empty_and_duplicates(self) -> None:
        with self.assertRaises(ValueError):
            MultiAxisTarget(())
        duplicate = AxisTarget(AxisName.SLIDE, 1.0)
        with self.assertRaises(ValueError):
            MultiAxisTarget((duplicate, duplicate))

    def test_public_dataclasses_are_frozen(self) -> None:
        target = AxisTarget(AxisName.SLIDE, 1.0)
        with self.assertRaises(FrozenInstanceError):
            target.position = 2.0  # type: ignore[misc]

    def test_startup_position_is_absent_from_normal_protocol(self) -> None:
        descriptor_fields = {item.name for item in fields(AxisDescriptor)}
        target_fields = {item.name for item in fields(AxisTarget)}
        multi_fields = {item.name for item in fields(MultiAxisTarget)}
        self.assertNotIn("startup_position", descriptor_fields)
        self.assertNotIn("startup_position", target_fields)
        self.assertNotIn("startup_position", multi_fields)


class ResultSemanticsTests(unittest.TestCase):
    def make_result(
        self,
        status: MotionCommandStatus,
        accepted: bool,
        completed: bool | None,
        error_code: MotionErrorCode | None = None,
    ) -> MotionCommandResult:
        return MotionCommandResult(
            "id",
            AxisName.SLIDE,
            status,
            accepted,
            completed,
            1.0,
            None,
            None,
            error_code,
            "test",
        )

    def test_accepted_moving_and_arrived_semantics(self) -> None:
        self.make_result(MotionCommandStatus.ACCEPTED, True, None)
        self.make_result(MotionCommandStatus.MOVING, True, None)
        self.make_result(MotionCommandStatus.ARRIVED, True, True)

    def test_rejected_and_terminal_failure_semantics(self) -> None:
        self.make_result(
            MotionCommandStatus.REJECTED,
            False,
            False,
            MotionErrorCode.INVALID_REQUEST,
        )
        self.make_result(
            MotionCommandStatus.TIMEOUT,
            True,
            False,
            MotionErrorCode.TIMEOUT,
        )

    def test_command_acceptance_cannot_claim_completion(self) -> None:
        with self.assertRaises(ValueError):
            self.make_result(MotionCommandStatus.ACCEPTED, True, True)

    def test_arrival_config_requires_explicit_valid_values(self) -> None:
        ArrivalConfig(0.1, 0.2, 0.01, 10.0)
        with self.assertRaises(ValueError):
            ArrivalConfig(0.0, 0.2, 0.01, 10.0)


if __name__ == "__main__":
    unittest.main()
