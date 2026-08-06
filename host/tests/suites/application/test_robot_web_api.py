from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
import threading
import unittest

from fastapi.testclient import TestClient

from application.robot_service import RobotServiceStateError
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.tray_workspace import TargetOutsideTrayWorkspace
from application.web_api import create_robot_web_app
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
)
from scripts import robot_web_api


@dataclass(frozen=True)
class FakeResult:
    operation: str
    completed: bool = True


class FakeRobotService:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[object, ...], dict[str, object]]] = []
        self.failures: dict[str, Exception] = {}
        self.shutdown_calls = 0
        self.startup_calls = 0
        self.backend_status: object | None = None
        self.capabilities = {
            "base_frame_motion": True,
            "axis_listing": True,
        }

    def _record(self, name: str, *args: object, **kwargs: object) -> None:
        self.calls.append((name, args, kwargs))
        failure = self.failures.get(name)
        if failure is not None:
            raise failure

    def status(self):
        self._record("status")
        return {
            "state": RobotServiceState.READY,
            "mode": RobotServiceMode.DRY_RUN,
            "backend_status": self.backend_status,
            "fault": None,
        }

    def startup(self):
        self._record("startup")
        self.startup_calls += 1

    def shutdown(self):
        self._record("shutdown")
        self.shutdown_calls += 1

    def stop(self):
        self._record("stop")

    def list_axes(self):
        self._record("list_axes")
        return (_descriptor(AxisName.SLIDE), _descriptor(AxisName.Z))

    def get_axis_state(self, axis: AxisName):
        self._record("get_axis_state", axis)
        return _state(axis)

    def move_axis_absolute(self, axis, position, **kwargs):
        self._record("move_axis_absolute", axis, position, **kwargs)
        return FakeResult("axis-absolute")

    def move_axis_relative(self, axis, delta, **kwargs):
        self._record("move_axis_relative", axis, delta, **kwargs)
        return FakeResult("axis-relative")

    def plan_base_target(self, target):
        self._record("plan_base_target", target)
        return FakeResult("base-plan")

    def move_base_target(self, target):
        self._record("move_base_target", target)
        return FakeResult("base-execute")

    def enable_joints(self):
        self._record("enable_joints")
        return FakeResult("joints-enable")

    def disable_joints(self):
        self._record("disable_joints")
        return FakeResult("joints-disable")

    def suction(self, action):
        self._record("suction", action)
        return FakeResult(f"suction-{action}")

    def request_observation(self):
        self._record("request_observation")
        return FakeResult("observe")

    def pick(self):
        self._record("pick")
        return FakeResult("pick")


class RobotWebApiTests(unittest.TestCase):
    def setUp(self) -> None:
        self.service = FakeRobotService()
        self.app = create_robot_web_app(self.service)
        self.client = TestClient(self.app, raise_server_exceptions=False)
        self.addCleanup(self.client.close)

    def test_basic_queries(self) -> None:
        self.assertEqual(self.client.get("/api/health").json(), {"ok": True})

        status = self.client.get("/api/status")
        self.assertEqual(status.status_code, 200)
        self.assertEqual(status.json()["state"], "ready")
        self.assertEqual(status.json()["mode"], "dry-run")

        capabilities = self.client.get("/api/capabilities")
        self.assertEqual(capabilities.status_code, 200)
        self.assertTrue(capabilities.json()["base_frame_motion"])

        axes = self.client.get("/api/axes")
        self.assertEqual(axes.status_code, 200)
        self.assertEqual(
            [item["name"] for item in axes.json()["axes"]],
            ["slide", "z"],
        )

        state = self.client.get("/api/axes/slide")
        self.assertEqual(state.status_code, 200)
        self.assertEqual(state.json()["axis"], "slide")
        self.assertEqual(state.json()["current_position"], 12.5)

    def test_lifecycle_routes_and_shutdown_are_idempotent(self) -> None:
        self.assertEqual(self.client.post("/api/startup").json(), {"ok": True})
        self.assertEqual(self.client.post("/api/stop").json(), {"ok": True})
        self.assertEqual(self.client.post("/api/shutdown").json(), {"ok": True})
        self.assertEqual(self.client.post("/api/shutdown").json(), {"ok": True})
        self.assertEqual(self.service.startup_calls, 1)
        self.assertEqual(self.service.shutdown_calls, 1)

    def test_axis_motion_requests_are_forwarded_exactly(self) -> None:
        absolute = self.client.post(
            "/api/axes/z/move-absolute",
            json={
                "position": -20.0,
                "velocity": 4.0,
                "acceleration": 8.0,
                "timeout_s": 3.0,
            },
        )
        self.assertEqual(absolute.status_code, 200)
        name, args, kwargs = self.service.calls[-1]
        self.assertEqual(name, "move_axis_absolute")
        self.assertEqual(args, (AxisName.Z, -20.0))
        self.assertEqual(
            kwargs,
            {"velocity": 4.0, "acceleration": 8.0, "timeout_s": 3.0},
        )

        relative = self.client.post(
            "/api/axes/rotation/move-relative",
            json={"delta": 5.0},
        )
        self.assertEqual(relative.status_code, 200)
        name, args, kwargs = self.service.calls[-1]
        self.assertEqual(name, "move_axis_relative")
        self.assertEqual(args, (AxisName.ROTATION, 5.0))
        self.assertEqual(
            kwargs,
            {"velocity": None, "acceleration": None, "timeout_s": None},
        )

    def test_base_joints_suction_observe_and_pick_are_forwarded(self) -> None:
        target = {"x_mm": 300, "y_mm": 400, "z_mm": 120, "yaw_deg": 0}
        for path, method_name in (
            ("/api/motion/base/plan", "plan_base_target"),
            ("/api/motion/base/execute", "move_base_target"),
        ):
            response = self.client.post(path, json=target)
            self.assertEqual(response.status_code, 200)
            name, args, kwargs = self.service.calls[-1]
            self.assertEqual(name, method_name)
            self.assertEqual(kwargs, {})
            self.assertEqual(
                (args[0].x_mm, args[0].y_mm, args[0].z_mm, args[0].yaw_deg),
                (300.0, 400.0, 120.0, 0.0),
            )

        routes = (
            ("/api/joints/enable", None, "enable_joints"),
            ("/api/joints/disable", None, "disable_joints"),
            ("/api/suction", {"action": "grip"}, "suction"),
            ("/api/vision/observe", None, "request_observation"),
            ("/api/pick", None, "pick"),
        )
        for path, body, expected_call in routes:
            response = self.client.post(path, json=body) if body else self.client.post(path)
            self.assertEqual(response.status_code, 200, path)
            self.assertEqual(self.service.calls[-1][0], expected_call)
        self.assertEqual(self.service.calls[-3][1], ("grip",))

    def test_invalid_requests_and_axis_return_400(self) -> None:
        invalid_body = self.client.post(
            "/api/axes/slide/move-absolute",
            json={"position": "not-a-number"},
        )
        self.assertEqual(invalid_body.status_code, 400)
        self.assertEqual(
            invalid_body.json()["error"]["type"],
            "RequestValidationError",
        )

        invalid_axis = self.client.get("/api/axes/not-an-axis")
        self.assertEqual(invalid_axis.status_code, 400)
        self.assertEqual(invalid_axis.json()["error"]["type"], "ValueError")

        wildcard_app_error = None
        try:
            create_robot_web_app(self.service, allowed_origins=["*"])
        except ValueError as exc:
            wildcard_app_error = exc
        self.assertIsNotNone(wildcard_app_error)

    def test_service_errors_are_mapped_without_sensitive_details(self) -> None:
        cases = (
            (
                RobotServiceStateError("move requires READY, got executing"),
                409,
                "RobotServiceStateError",
                True,
            ),
            (
                TargetOutsideTrayWorkspace("target is outside tray"),
                422,
                "TargetOutsideTrayWorkspace",
                True,
            ),
            (
                RuntimeError("serial /dev/tty.secret disconnected"),
                503,
                "RuntimeError",
                False,
            ),
            (
                KeyError("/private/secret/path"),
                500,
                "InternalServerError",
                False,
            ),
        )
        for error, status_code, error_type, exposes_message in cases:
            with self.subTest(error=type(error).__name__):
                self.service.failures["move_base_target"] = error
                log_context = (
                    self.assertLogs("application.web_api", level="WARNING")
                    if status_code >= 500
                    else nullcontext()
                )
                with log_context:
                    response = self.client.post(
                        "/api/motion/base/execute",
                        json={"x_mm": 300, "y_mm": 400, "z_mm": 120},
                    )
                self.assertEqual(response.status_code, status_code)
                payload = response.json()["error"]
                self.assertEqual(payload["type"], error_type)
                self.assertEqual(
                    "target is outside tray" in payload["message"]
                    or "got executing" in payload["message"],
                    exposes_message,
                )
                self.assertNotIn("tty.secret", payload["message"])
                self.assertNotIn("/private/secret", payload["message"])

    def test_openapi_and_cors_are_available(self) -> None:
        schema = self.client.get("/openapi.json")
        self.assertEqual(schema.status_code, 200)
        self.assertIn("/api/status", schema.json()["paths"])

        response = self.client.options(
            "/api/status",
            headers={
                "Origin": "http://localhost:5173",
                "Access-Control-Request-Method": "GET",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            response.headers["access-control-allow-origin"],
            "http://localhost:5173",
        )

    def test_status_redacts_local_paths_and_serial_names(self) -> None:
        self.service.backend_status = (
            "unavailable: /dev/tty.secret; /Users/operator/private.json; COM12"
        )
        response = self.client.get("/api/status")
        self.assertEqual(response.status_code, 200)
        serialized = response.json()["backend_status"]
        self.assertNotIn("tty.secret", serialized)
        self.assertNotIn("/Users/operator", serialized)
        self.assertNotIn("COM12", serialized)


class ConcurrentFakeRobotService(FakeRobotService):
    def __init__(self) -> None:
        super().__init__()
        self.move_entered = threading.Event()
        self.release_move = threading.Event()
        self.stop_entered = threading.Event()
        self._operation_lock = threading.Lock()
        self._moving = False
        self.backend_submits = 0

    def status(self):
        self._record("status")
        with self._operation_lock:
            state = RobotServiceState.EXECUTING if self._moving else RobotServiceState.READY
        return {"state": state, "mode": RobotServiceMode.EXECUTE}

    def move_axis_absolute(self, axis, position, **kwargs):
        del axis, position, kwargs
        with self._operation_lock:
            if self._moving:
                raise RobotServiceStateError("raw axis motion requires READY, got executing")
            self._moving = True
            self.backend_submits += 1
        self.move_entered.set()
        self.release_move.wait(timeout=3.0)
        with self._operation_lock:
            self._moving = False
        return FakeResult("axis-absolute")

    def stop(self):
        self.stop_entered.set()
        self.release_move.set()


class RobotWebApiConcurrencyTests(unittest.TestCase):
    def test_status_stop_and_busy_rejection_remain_concurrent(self) -> None:
        service = ConcurrentFakeRobotService()
        client = TestClient(
            create_robot_web_app(service),
            raise_server_exceptions=False,
        )
        self.addCleanup(client.close)
        first_response: list[object] = []
        worker = threading.Thread(
            target=lambda: first_response.append(
                client.post(
                    "/api/axes/slide/move-absolute",
                    json={"position": 10.0},
                )
            )
        )
        worker.start()
        self.assertTrue(service.move_entered.wait(timeout=1.0))

        status = client.get("/api/status")
        self.assertEqual(status.status_code, 200)
        self.assertEqual(status.json()["state"], "executing")

        second = client.post(
            "/api/axes/z/move-absolute",
            json={"position": -10.0},
        )
        self.assertEqual(second.status_code, 409)
        self.assertEqual(service.backend_submits, 1)

        stopped = client.post("/api/stop")
        self.assertEqual(stopped.status_code, 200)
        self.assertTrue(service.stop_entered.is_set())
        worker.join(timeout=2.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(first_response[0].status_code, 200)
        self.assertEqual(service.backend_submits, 1)


class RobotWebApiScriptTests(unittest.TestCase):
    def test_main_reuses_cli_factory_defaults_and_shuts_down_once(self) -> None:
        service = FakeRobotService()
        observed: dict[str, object] = {}

        def service_factory(args):
            observed["mode"] = args.mode
            return service

        def runner(app, *, host, port):
            observed["service"] = app.state.robot_service
            observed["host"] = host
            observed["port"] = port
            with TestClient(app, raise_server_exceptions=False) as client:
                self.assertEqual(client.get("/api/health").status_code, 200)
                self.assertEqual(client.post("/api/shutdown").status_code, 200)

        result = robot_web_api.main(
            [],
            service_factory=service_factory,
            uvicorn_runner=runner,
        )
        self.assertEqual(result, 0)
        self.assertEqual(observed["mode"], "read-only")
        self.assertIs(observed["service"], service)
        self.assertEqual(observed["host"], "127.0.0.1")
        self.assertEqual(observed["port"], 8000)
        self.assertEqual(service.startup_calls, 0)
        self.assertEqual(service.shutdown_calls, 1)


def _descriptor(axis: AxisName) -> AxisDescriptor:
    return AxisDescriptor(
        name=axis,
        display_name=axis.value.title(),
        kind=AxisKind.LINEAR,
        position_unit="mm",
        velocity_unit="mm/s",
        acceleration_unit="mm/s^2",
        minimum_position=-200.0,
        maximum_position=800.0,
        capabilities=AxisCapabilities(True, True, True, True, True, True, True),
    )


def _state(axis: AxisName) -> AxisState:
    return AxisState(
        axis=axis,
        connected=True,
        enabled=True,
        busy=False,
        homed=True,
        position_valid=True,
        current_position=12.5,
        position_unit="mm",
        faulted=False,
        fault_code=None,
        fault_message=None,
    )


if __name__ == "__main__":
    unittest.main()
