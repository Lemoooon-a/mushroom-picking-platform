"""上部运动后端可离线装配且能力可查询的 smoke test。"""

from __future__ import annotations

import unittest

from config.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from drivers.stm32_motion import STM32MotionClient
from motion.capabilities import UpperMotionBackends
from robot.feetech_rotation import FeetechRotationAxis, FeetechRotationConfig
from robot.joint import CanRotaryJoint


class EmptyLineTransport:
    def write_line(self, _line: str) -> None: pass
    def read_line(self) -> str | None: return None
    def close(self) -> None: pass


class FakeMotorDriver:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id


class UpperMotionSmokeTests(unittest.TestCase):
    def test_all_backends_construct_without_hardware_io(self) -> None:
        stm32 = STM32MotionClient(EmptyLineTransport())
        shoulder = CanRotaryJoint(
            FakeMotorDriver(SHOULDER_JOINT_CONFIG.motor_id),  # type: ignore[arg-type]
            SHOULDER_JOINT_CONFIG,
        )
        elbow = CanRotaryJoint(
            FakeMotorDriver(ELBOW_JOINT_CONFIG.motor_id),  # type: ignore[arg-type]
            ELBOW_JOINT_CONFIG,
        )
        rotation = FeetechRotationAxis(
            object(),  # type: ignore[arg-type]
            FeetechRotationConfig(
                name="rotation-test-only",
                servo_id=1,
                counts_per_turn=4096,
                zero_raw=0,
                direction_sign=1,
                min_position_rad=-1,
                max_position_rad=1,
                max_speed_raw=1000,
            ),
        )
        backends = UpperMotionBackends(
            slide=stm32,
            z=stm32,
            shoulder=shoulder,
            elbow=elbow,
            rotation=rotation,
            vacuum=stm32,
        )
        matrix = backends.capability_matrix()
        self.assertEqual(set(matrix), {"slide", "z", "shoulder", "elbow", "rotation", "vacuum"})
        self.assertTrue(matrix["slide"].home)
        self.assertFalse(matrix["shoulder"].arrival_event)
        self.assertTrue(matrix["rotation"].disable)
        self.assertFalse(matrix["rotation"].stop)
        self.assertTrue(matrix["vacuum"].arrival_event)


if __name__ == "__main__":
    unittest.main()
