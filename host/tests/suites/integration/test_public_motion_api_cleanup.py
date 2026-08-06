"""旧运动 façade 不得重新成为仓库公共入口。"""

from __future__ import annotations

import importlib.util
import unittest

import application
import motion
from bootstrap import UpperMotionRuntime


class PublicMotionApiCleanupTests(unittest.TestCase):
    def test_legacy_client_modules_and_symbols_are_absent(self) -> None:
        self.assertIsNone(importlib.util.find_spec("motion.client_facades"))
        self.assertIsNone(importlib.util.find_spec("motion.client_interfaces"))
        removed = {
            "FrontendMotionInterface",
            "KinematicsMotionInterface",
            "FrontendMotionFacade",
            "KinematicsMotionFacade",
        }
        self.assertTrue(removed.isdisjoint(motion.__all__))
        for name in removed:
            self.assertFalse(hasattr(motion, name))

    def test_runtime_has_no_legacy_facade_properties(self) -> None:
        self.assertNotIn("frontend_motion", UpperMotionRuntime.__dict__)
        self.assertNotIn("kinematics_motion", UpperMotionRuntime.__dict__)

    def test_application_public_entry_is_robot_service(self) -> None:
        self.assertIn("MushroomRobotService", application.__all__)
        self.assertNotIn("MushroomRobotController", application.__all__)

    def test_calibration_modules_still_import(self) -> None:
        from scripts import calibrate_base_slide_frame, verify_base_slide_frame

        self.assertIsNotNone(calibrate_base_slide_frame.capture_and_calibrate)
        self.assertIsNotNone(verify_base_slide_frame.capture_and_verify)


if __name__ == "__main__":
    unittest.main()
