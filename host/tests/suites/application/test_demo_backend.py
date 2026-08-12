from __future__ import annotations

import unittest

from application.demo_backend import DemoFlowApplicationBackend


class _Runtime:
    def __init__(self, calls: list[str]) -> None:
        self.calls = calls

    def open(self) -> None:
        self.calls.append("open")

    def close(self) -> None:
        self.calls.append("close")


class _Flow:
    def __init__(self, calls: list[str]) -> None:
        self.calls = calls
        self.last_stop_report = object()

    def startup(self) -> None:
        self.calls.append("startup")
        raise RuntimeError("startup failed")

    def stop(self) -> None:
        self.calls.append("stop")


class DemoFlowApplicationBackendTests(unittest.TestCase):
    def test_startup_failure_stops_before_transport_close(self) -> None:
        calls: list[str] = []
        flow = _Flow(calls)
        backend = DemoFlowApplicationBackend(
            runtime=_Runtime(calls),
            flow=flow,
        )

        with self.assertRaisesRegex(RuntimeError, "startup failed") as context:
            backend.startup()

        self.assertEqual(calls, ["open", "startup", "stop", "close"])
        self.assertIs(context.exception.stop_report, flow.last_stop_report)


if __name__ == "__main__":
    unittest.main()
