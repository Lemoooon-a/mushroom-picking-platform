from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
import threading
import unittest

from fastapi.testclient import TestClient

from application.robot_service import RobotServiceStateError
from application.robot_service import ResolvedCameraPoint
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.scan_pick import ScanAndPickResult, ScanPositionResult
from application.tray_workspace import TargetOutsideTrayWorkspace
from application.web_api import create_robot_web_app
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import (
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
)
from scripts import robot_web_api
from vision.observation import CaptureMotionState, Vector3, VisionTargetObservation


@dataclass(frozen=True)
class FakeResult:
    operation: str
    completed: bool = True
    stages: tuple[object, ...] = ()


class FakeRobotService:
    def __init__(self) -> None:
        self.mode = RobotServiceMode.DRY_RUN
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

    def get_current_tcp_pose(self):
        self._record("get_current_tcp_pose")
        return {
            "x_mm": 250.0,
            "y_mm": 200.0,
            "z_mm": 200.0,
            "yaw_deg": 0.0,
            "frame_id": "base",
        }

    def return_to_startup(self):
        self._record("return_to_startup")
        return FakeResult("return-to-startup")

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
        return VisionTargetObservation(
            request_id="capture-000001",
            frame_id="camera_color_optical_frame",
            timestamp=100.0,
            position_mm=Vector3(20.0, -15.0, 450.0),
            confidence=0.95,
            target_id=None,
            orientation=None,
            capture_axis_state=RobotAxisState(1.0, -2.0, 3.0, 4.0, 5.0),
            capture_motion_state=CaptureMotionState.STATIONARY,
        )

    def resolve_camera_point(self, x_mm, y_mm, z_mm, **kwargs):
        self._record("resolve_camera_point", x_mm, y_mm, z_mm, **kwargs)
        return ResolvedCameraPoint(
            camera_point_mm=(x_mm, y_mm, z_mm),
            base_point_mm=(175.0, 222.5, 34.85),
            frame_id=kwargs["frame_id"],
            tool_camera_source="manual_measurement",
            tool_camera_validated=False,
            raw_base_point_mm=(185.0, 212.5, 44.85),
            target_compensation_base_mm=(-10.0, 10.0, -10.0),
        )

    def pick(self):
        self._record("pick")
        return FakeResult("pick")

    def move_to_scan_position(self, scan_index):
        self._record("move_to_scan_position", scan_index)
        return FakeResult("scan-position-move")

    def pick_one_at_scan_position(self, scan_index):
        self._record("pick_one_at_scan_position", scan_index)
        return ScanAndPickResult(
            "completed",
            (
                ScanPositionResult(
                    scan_index,
                    1,
                    1,
                    "picked_and_placed_unverified",
                ),
            ),
            1,
        )

    def scan_and_pick(self):
        self._record("scan_and_pick")
        return ScanAndPickResult(
            "completed",
            (ScanPositionResult(1, 2, 2, "no_target"),),
            2,
        )


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

        current = self.client.get("/api/motion/base/current")
        self.assertEqual(current.status_code, 200)
        self.assertEqual(
            current.json(),
            {
                "x_mm": 250.0,
                "y_mm": 200.0,
                "z_mm": 200.0,
                "yaw_deg": 0.0,
                "frame_id": "base",
            },
        )
        self.assertEqual(self.service.calls[-1][0], "get_current_tcp_pose")

        returned = self.client.post("/api/motion/return-to-startup")
        self.assertEqual(returned.status_code, 200)
        self.assertEqual(returned.json()["operation"], "return-to-startup")
        self.assertEqual(self.service.calls[-1][0], "return_to_startup")

        routes = (
            ("/api/joints/enable", None, "enable_joints"),
            ("/api/joints/disable", None, "disable_joints"),
            ("/api/suction", {"action": "grip"}, "suction"),
            ("/api/vision/observe", None, "request_observation"),
            ("/api/pick", None, "pick"),
            ("/api/scan-pick", None, "scan_and_pick"),
        )
        for path, body, expected_call in routes:
            response = self.client.post(path, json=body) if body else self.client.post(path)
            self.assertEqual(response.status_code, 200, path)
            self.assertEqual(self.service.calls[-1][0], expected_call)
        suction_call = next(
            call for call in self.service.calls if call[0] == "suction"
        )
        self.assertEqual(suction_call[1], ("grip",))

    def test_scan_position_routes_are_forwarded_and_validate_integer_path(self) -> None:
        moved = self.client.post("/api/scan-positions/3/move")
        self.assertEqual(moved.status_code, 200)
        self.assertEqual(moved.json()["operation"], "scan-position-move")
        self.assertEqual(
            self.service.calls[-1],
            ("move_to_scan_position", (3,), {}),
        )

        picked = self.client.post("/api/scan-positions/3/pick-one")
        self.assertEqual(picked.status_code, 200)
        self.assertEqual(picked.json()["total_picked"], 1)
        self.assertEqual(
            picked.json()["visited_scan_positions"][0]["final_reason"],
            "picked_and_placed_unverified",
        )
        self.assertEqual(
            self.service.calls[-1],
            ("pick_one_at_scan_position", (3,), {}),
        )

        for path in (
            "/api/scan-positions/0/move",
            "/api/scan-positions/9/pick-one",
            "/api/scan-positions/not-an-integer/move",
        ):
            with self.subTest(path=path):
                invalid = self.client.post(path)
                self.assertEqual(invalid.status_code, 400)
                self.assertEqual(
                    invalid.json()["error"]["type"],
                    "RequestValidationError",
                )

    def test_vision_plan_observes_resolves_capture_snapshot_and_only_plans(self) -> None:
        self.service.calls.clear()

        response = self.client.post("/api/vision/plan")

        self.assertEqual(response.status_code, 200)
        payload = response.json()
        self.assertEqual(payload["request_id"], "capture-000001")
        self.assertEqual(
            payload["camera"]["position_mm"],
            {"x": 20.0, "y": -15.0, "z": 450.0},
        )
        self.assertEqual(
            payload["capture_joint_state"],
            {
                "slide_mm": 1.0,
                "z_mm": -2.0,
                "shoulder_deg": 3.0,
                "elbow_deg": 4.0,
                "rotation_deg": 5.0,
            },
        )
        self.assertEqual(
            payload["base"]["position_mm"],
            {"x": 175.0, "y": 222.5, "z": 34.85},
        )
        self.assertEqual(
            payload["base"]["raw_position_mm"],
            {"x": 185.0, "y": 212.5, "z": 44.85},
        )
        self.assertEqual(
            payload["base"]["target_compensation_base_mm"],
            {"x": -10.0, "y": 10.0, "z": -10.0},
        )
        self.assertFalse(payload["base"]["tool_camera_validated"])
        self.assertTrue(payload["planner"]["succeeded"])
        self.assertEqual(
            [call[0] for call in self.service.calls],
            ["request_observation", "resolve_camera_point", "plan_base_target"],
        )
        planned_target = self.service.calls[-1][1][0]
        self.assertEqual(
            (
                planned_target.x_mm,
                planned_target.y_mm,
                planned_target.z_mm,
                planned_target.yaw_deg,
            ),
            (175.0, 222.5, 34.85, None),
        )
        self.assertFalse(
            any(call[0] == "move_base_target" for call in self.service.calls)
        )

    def test_planning_rejection_exposes_existing_stage_reason(self) -> None:
        self.service.failures["plan_base_target"] = TargetOutsideTrayWorkspace(
            "target is outside tray"
        )

        response = self.client.post("/api/vision/plan")

        self.assertEqual(response.status_code, 422)
        self.assertEqual(
            response.json()["error"]["rejection_reason"],
            "outside_tray_workspace",
        )

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
        self.assertIn("/api/vision/plan", schema.json()["paths"])
        self.assertIn(
            "/api/scan-positions/{scan_index}/move",
            schema.json()["paths"],
        )
        self.assertIn(
            "/api/scan-positions/{scan_index}/pick-one",
            schema.json()["paths"],
        )
        self.assertIn("/api/scan-pick", schema.json()["paths"])

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
        self.assertEqual(observed["host"], "172.20.10.3")
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
