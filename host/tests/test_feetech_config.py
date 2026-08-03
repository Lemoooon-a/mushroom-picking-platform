"""SM45BL-C001 型号 profile 的离线测试。"""

from __future__ import annotations

import math
import unittest

from config.feetech import (
    END_EFFECTOR_ROTATION_CONFIG,
    END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW,
    END_EFFECTOR_ROTATION_POSITIVE_DIRECTION,
    FEETECH_MODEL_PROFILES,
    SM45BL_C001_PROFILE,
)


class SM45BLC001ProfileTests(unittest.TestCase):
    def test_confirmed_model_constants(self) -> None:
        profile = SM45BL_C001_PROFILE
        self.assertEqual(profile.key, "sm45bl-c001")
        self.assertEqual(profile.model, "SM-45BL-C001")
        self.assertEqual(profile.protocol, "Feetech custom serial")
        self.assertEqual(profile.transport, "RS-485 half-duplex")
        self.assertEqual(profile.default_baudrate, 115200)
        self.assertEqual(profile.counts_per_turn, 4096)
        self.assertTrue(profile.adapter_auto_direction)
        self.assertEqual(profile.registers.goal_position, 0x2A)
        self.assertEqual(profile.registers.present_position, 0x38)
        self.assertIs(FEETECH_MODEL_PROFILES[profile.key], profile)

    def test_profile_combines_with_mechanical_calibration(self) -> None:
        config = SM45BL_C001_PROFILE.make_rotation_config(
            name="rotation",
            servo_id=7,
            zero_raw=2048,
            direction_sign=-1,
            min_position_rad=-1.0,
            max_position_rad=1.0,
            max_speed_raw=200,
        )
        self.assertEqual(config.servo_id, 7)
        self.assertEqual(config.counts_per_turn, 4096)
        self.assertEqual(config.zero_raw, 2048)
        self.assertEqual(config.direction_sign, -1)
        self.assertIs(config.registers, SM45BL_C001_PROFILE.registers)

    def test_project_rotation_config_is_fixed_to_confirmed_parameters(self) -> None:
        config = END_EFFECTOR_ROTATION_CONFIG
        self.assertEqual(config.name, "end_effector_rotation")
        self.assertEqual(config.servo_id, 1)
        self.assertEqual(config.counts_per_turn, 4096)
        self.assertEqual(config.zero_raw, 2130)
        self.assertEqual(config.direction_sign, 1)
        self.assertAlmostEqual(config.min_position_rad, math.radians(-45.0))
        self.assertAlmostEqual(config.max_position_rad, math.radians(45.0))
        self.assertEqual(config.max_speed_raw, 500)
        self.assertEqual(END_EFFECTOR_ROTATION_DEFAULT_SPEED_RAW, 500)
        self.assertEqual(END_EFFECTOR_ROTATION_POSITIVE_DIRECTION, "+X")


if __name__ == "__main__":
    unittest.main()
