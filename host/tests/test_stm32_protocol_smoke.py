"""STM32 v2 串口 smoke 脚本的无硬件安全门测试。"""

from __future__ import annotations

from contextlib import redirect_stderr
from io import StringIO
import unittest

from scripts.stm32_protocol_smoke import build_parser, main


class STM32ProtocolSmokeTests(unittest.TestCase):
    def test_default_arguments_do_not_request_motion(self) -> None:
        args = build_parser().parse_args(["TEST_PORT"])
        self.assertFalse(args.allow_motion)
        self.assertIsNone(args.home)
        self.assertIsNone(args.position_mm)

    def test_motion_options_require_explicit_allow_motion(self) -> None:
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
            main(["TEST_PORT", "--home", "z"])
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
