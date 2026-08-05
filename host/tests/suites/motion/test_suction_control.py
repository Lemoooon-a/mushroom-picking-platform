"""统一吸盘语义适配的离线测试。"""

from __future__ import annotations

from types import SimpleNamespace
import unittest

from motion.suction import STM32SuctionControl, SuctionMode


class FakeSTM32Client:
    def __init__(self) -> None:
        self.calls: list[str] = []
        self.raw = SimpleNamespace(
            state=0,
            pump_on=False,
            release_open=False,
            busy=False,
            fault=0,
        )

    def suction_grip(self) -> None:
        self.calls.append("SU")
        self.raw = SimpleNamespace(
            state=1,
            pump_on=True,
            release_open=False,
            busy=False,
            fault=0,
        )

    def suction_release(self) -> None:
        self.calls.append("SR")
        self.raw = SimpleNamespace(
            state=3,
            pump_on=False,
            release_open=True,
            busy=False,
            fault=0,
        )

    def suction_idle(self) -> None:
        self.calls.append("SX")
        self.raw = SimpleNamespace(
            state=0,
            pump_on=False,
            release_open=False,
            busy=False,
            fault=0,
        )

    def query_suction(self) -> object:
        self.calls.append("SQ")
        return self.raw


class STM32SuctionControlTests(unittest.TestCase):
    def test_all_actions_reuse_one_client_and_query_acknowledged_outputs(self) -> None:
        client = FakeSTM32Client()
        suction = STM32SuctionControl(client)

        grip = suction.grip()
        release = suction.release()
        idle = suction.idle()

        self.assertEqual(client.calls, ["SU", "SQ", "SR", "SQ", "SX", "SQ"])
        self.assertEqual(grip.mode, SuctionMode.GRIP)
        self.assertTrue(grip.pump_on)
        self.assertEqual(release.mode, SuctionMode.RELEASE)
        self.assertTrue(release.release_open)
        self.assertEqual(idle.mode, SuctionMode.IDLE)

    def test_status_never_claims_physical_vacuum_verification(self) -> None:
        status = STM32SuctionControl(FakeSTM32Client()).get_status()
        self.assertTrue(status.command_acknowledged)
        self.assertFalse(status.physically_verified)
        self.assertIsNone(status.vacuum_detected)

    def test_unknown_firmware_state_is_preserved_as_unknown(self) -> None:
        client = FakeSTM32Client()
        client.raw.state = 99
        status = STM32SuctionControl(client).get_status()
        self.assertEqual(status.mode, SuctionMode.UNKNOWN)
        self.assertEqual(status.raw_state, 99)


if __name__ == "__main__":
    unittest.main()
