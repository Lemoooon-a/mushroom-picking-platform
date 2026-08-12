"""MushroomRobotService 的最小 FastAPI 薄适配层。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import fields, is_dataclass
from enum import Enum
import logging
import math
from numbers import Integral, Real
import re
import threading
from typing import Annotated, Literal

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat

from application.motion_target import BaseToolTarget
from application.robot_service import (
    MushroomRobotService,
    RobotServiceCapabilityError,
    RobotServiceError,
    RobotServiceStateError,
)
from application.runtime_state import RobotServiceMode
from motion.unified_protocol import AxisName


LOGGER = logging.getLogger(__name__)

DEFAULT_CORS_ORIGINS = (
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    "http://localhost:5173",
    "http://127.0.0.1:5173",
)

_PLANNING_ERROR_BASE_NAMES = frozenset(
    {
        "BaseFrameSolverError",
        "BaseMovePlanningError",
        "KinematicsError",
        "PickPlanningError",
        "TargetOutsideTrayWorkspace",
        "VisionTargetResolutionError",
    }
)
_CONFLICT_ERROR_BASE_NAMES = frozenset({"MotionAuthorizationError"})
_SENSITIVE_POSIX_PATH = re.compile(
    r"(?<![\w:])/(?:Users|home|private|Volumes|dev|var|tmp)(?:/[^\s,;\"']*)?"
)
_SENSITIVE_WINDOWS_PATH = re.compile(r"\b[A-Za-z]:\\[^\s,;\"']+")
_SERIAL_PORT_NAME = re.compile(r"\bCOM\d+\b", re.IGNORECASE)

PositiveFiniteFloat = Annotated[FiniteFloat, Field(gt=0)]


class _RequestModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class BaseTargetRequest(_RequestModel):
    x_mm: FiniteFloat
    y_mm: FiniteFloat
    z_mm: FiniteFloat
    yaw_deg: FiniteFloat | None = None


class AxisAbsoluteMoveRequest(_RequestModel):
    position: FiniteFloat
    velocity: PositiveFiniteFloat | None = None
    acceleration: PositiveFiniteFloat | None = None
    timeout_s: PositiveFiniteFloat | None = None


class AxisRelativeMoveRequest(_RequestModel):
    delta: FiniteFloat
    velocity: PositiveFiniteFloat | None = None
    acceleration: PositiveFiniteFloat | None = None
    timeout_s: PositiveFiniteFloat | None = None


class SuctionRequest(_RequestModel):
    action: Literal["grip", "release", "idle"]


class _ResponseEncodingError(Exception):
    """Service 返回了 Web 边界不允许公开的对象。"""


class _ShutdownOnce:
    def __init__(self, service: MushroomRobotService) -> None:
        self._service = service
        self._lock = threading.Lock()
        self._called = False
        self._error: Exception | None = None

    def __call__(self) -> None:
        with self._lock:
            if self._called:
                if self._error is not None:
                    raise self._error
                return
            self._called = True
            try:
                self._service.shutdown()
            except Exception as exc:
                self._error = exc
                raise


def create_robot_web_app(
    service: MushroomRobotService,
    *,
    allowed_origins: Sequence[str] | None = None,
) -> FastAPI:
    """为外部提供的单一 Service 实例创建 HTTP JSON 适配器。"""

    origins = _normalize_origins(allowed_origins)
    app = FastAPI(
        title="Mushroom Robot Service API",
        version="1.0.0",
        description=(
            "MushroomRobotService 的同步薄适配层。启动 Web 服务不会自动执行 "
            "startup、home 或运动。"
        ),
    )
    app.state.robot_service = service
    app.state.shutdown_service_once = _ShutdownOnce(service)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(origins),
        allow_credentials=False,
        allow_methods=["GET", "POST"],
        allow_headers=["Content-Type"],
    )

    @app.exception_handler(RequestValidationError)
    async def request_validation_error(
        request: Request,
        exc: RequestValidationError,
    ) -> JSONResponse:
        del request
        return _error_payload(
            status_code=400,
            error_type=type(exc).__name__,
            message="Request body or path parameters are invalid.",
        )

    @app.exception_handler(Exception)
    async def unexpected_framework_error(
        request: Request,
        exc: Exception,
    ) -> JSONResponse:
        LOGGER.exception(
            "Unhandled Web API exception for %s %s",
            request.method,
            request.url.path,
        )
        return _error_payload(
            status_code=500,
            error_type="InternalServerError",
            message="Internal server error.",
        )

    @app.get("/api/health", tags=["status"])
    def health() -> JSONResponse:
        return _success({"ok": True})

    @app.get("/api/status", tags=["status"])
    def status() -> JSONResponse:
        return _invoke(service.status)

    @app.get("/api/capabilities", tags=["status"])
    def capabilities() -> JSONResponse:
        return _invoke(lambda: service.capabilities)

    @app.post("/api/startup", tags=["lifecycle"])
    def startup() -> JSONResponse:
        return _invoke(service.startup)

    @app.post("/api/shutdown", tags=["lifecycle"])
    def shutdown() -> JSONResponse:
        return _invoke(app.state.shutdown_service_once)

    @app.post("/api/stop", tags=["lifecycle"])
    def stop() -> JSONResponse:
        return _invoke(service.stop)

    @app.get("/api/axes", tags=["axes"])
    def axes() -> JSONResponse:
        return _invoke(lambda: {"axes": service.list_axes()})

    @app.get("/api/axes/{axis}", tags=["axes"])
    def axis_state(axis: str) -> JSONResponse:
        return _invoke(lambda: service.get_axis_state(_axis_name(axis)))

    @app.post("/api/axes/{axis}/move-absolute", tags=["axes"])
    def move_axis_absolute(
        axis: str,
        request: AxisAbsoluteMoveRequest,
    ) -> JSONResponse:
        return _invoke(
            lambda: service.move_axis_absolute(
                _axis_name(axis),
                request.position,
                velocity=request.velocity,
                acceleration=request.acceleration,
                timeout_s=request.timeout_s,
            )
        )

    @app.post("/api/axes/{axis}/move-relative", tags=["axes"])
    def move_axis_relative(
        axis: str,
        request: AxisRelativeMoveRequest,
    ) -> JSONResponse:
        return _invoke(
            lambda: service.move_axis_relative(
                _axis_name(axis),
                request.delta,
                velocity=request.velocity,
                acceleration=request.acceleration,
                timeout_s=request.timeout_s,
            )
        )

    @app.post("/api/motion/base/plan", tags=["base motion"])
    def plan_base(request: BaseTargetRequest) -> JSONResponse:
        return _invoke(service.plan_base_target, _base_target(request))

    @app.post("/api/motion/base/execute", tags=["base motion"])
    def execute_base(request: BaseTargetRequest) -> JSONResponse:
        return _invoke(service.move_base_target, _base_target(request))

    @app.get("/api/motion/base/current", tags=["base motion"])
    def current_base_tcp() -> JSONResponse:
        return _invoke(service.get_current_tcp_pose)

    @app.post("/api/motion/return-to-startup", tags=["base motion"])
    def return_to_startup() -> JSONResponse:
        return _invoke(service.return_to_startup)

    @app.post("/api/joints/enable", tags=["joints"])
    def enable_joints() -> JSONResponse:
        return _invoke(service.enable_joints)

    @app.post("/api/joints/disable", tags=["joints"])
    def disable_joints() -> JSONResponse:
        return _invoke(service.disable_joints)

    @app.post("/api/suction", tags=["suction"])
    def suction(request: SuctionRequest) -> JSONResponse:
        return _invoke(service.suction, request.action)

    @app.post("/api/vision/observe", tags=["vision and pick"])
    def observe() -> JSONResponse:
        return _invoke(service.request_observation)

    @app.post("/api/vision/plan", tags=["vision and pick"])
    def plan_vision_target() -> JSONResponse:
        return _invoke(_observe_and_plan_vision_target, service)

    @app.post("/api/pick", tags=["vision and pick"])
    def pick() -> JSONResponse:
        return _invoke(service.pick)

    @app.post("/api/scan-pick", tags=["vision and pick"])
    def scan_pick() -> JSONResponse:
        return _invoke(service.scan_and_pick)

    return app


def _observe_and_plan_vision_target(
    service: MushroomRobotService,
) -> dict[str, object]:
    """拍照、按 capture 快照转换坐标并只做 Base 规划。"""

    observation = service.request_observation()
    resolved = service.resolve_camera_point(
        observation.position_mm.x,
        observation.position_mm.y,
        observation.position_mm.z,
        frame_id=observation.frame_id,
        capture_axis_state=observation.capture_axis_state,
    )
    if (
        not resolved.tool_camera_validated
        and service.mode is not RobotServiceMode.DRY_RUN
    ):
        raise RobotServiceCapabilityError(
            "Provisional tool_T_camera is allowed only for dry-run vision planning."
        )
    base_x, base_y, base_z = resolved.base_point_mm
    raw_base_x, raw_base_y, raw_base_z = resolved.raw_base_point_mm
    compensation_x, compensation_y, compensation_z = (
        resolved.target_compensation_base_mm
    )
    camera_compensation_x, camera_compensation_y, camera_compensation_z = (
        resolved.target_compensation_camera_mm
    )
    plan = service.plan_base_target(
        BaseToolTarget(base_x, base_y, base_z, yaw_deg=None)
    )
    stages = getattr(plan, "stages", ())
    final_solution = stages[-1].solution if stages else None
    return {
        "request_id": observation.request_id,
        "camera": {
            "frame_id": observation.frame_id,
            "position_mm": observation.position_mm,
            "target_compensation_camera_mm": {
                "x": camera_compensation_x,
                "y": camera_compensation_y,
                "z": camera_compensation_z,
            },
            "confidence": observation.confidence,
            "timestamp": observation.timestamp,
            "target_id": observation.target_id,
            "orientation": observation.orientation,
        },
        "capture_joint_state": observation.capture_axis_state,
        "base": {
            "frame_id": "base",
            "raw_position_mm": {
                "x": raw_base_x,
                "y": raw_base_y,
                "z": raw_base_z,
            },
            "target_compensation_base_mm": {
                "x": compensation_x,
                "y": compensation_y,
                "z": compensation_z,
            },
            "position_mm": {
                "x": base_x,
                "y": base_y,
                "z": base_z,
            },
            "tool_camera_source": resolved.tool_camera_source,
            "tool_camera_validated": resolved.tool_camera_validated,
            "transform_status": resolved.transform_status,
        },
        "planner": {
            "succeeded": True,
            "five_axis_solution": final_solution,
            "plan": plan,
        },
    }


def _normalize_origins(origins: Sequence[str] | None) -> tuple[str, ...]:
    if isinstance(origins, str):
        raise TypeError("allowed_origins must be a sequence of origins, not a string")
    selected = DEFAULT_CORS_ORIGINS if origins is None else tuple(origins)
    normalized: list[str] = []
    for origin in selected:
        if not isinstance(origin, str) or not origin.strip():
            raise ValueError("CORS origins must be non-empty strings")
        value = origin.strip().rstrip("/")
        if value == "*":
            raise ValueError("wildcard CORS origin is not allowed")
        if value not in normalized:
            normalized.append(value)
    return tuple(normalized)


def _axis_name(value: str) -> AxisName:
    try:
        return AxisName(value)
    except ValueError as exc:
        allowed = ", ".join(axis.value for axis in AxisName)
        raise ValueError(f"unknown axis {value!r}; expected one of: {allowed}") from exc


def _base_target(request: BaseTargetRequest) -> BaseToolTarget:
    return BaseToolTarget(
        request.x_mm,
        request.y_mm,
        request.z_mm,
        request.yaw_deg,
    )


def _invoke(function: Callable[..., object], *args: object, **kwargs: object) -> JSONResponse:
    try:
        return _success(function(*args, **kwargs))
    except Exception as exc:
        return _exception_response(exc)


def _success(value: object) -> JSONResponse:
    content = {"ok": True} if value is None else _jsonable(value)
    return JSONResponse(status_code=200, content=content)


def _exception_response(exc: Exception) -> JSONResponse:
    status_code = _exception_status(exc)
    if status_code == 500:
        LOGGER.exception("Unexpected exception in Robot Web API", exc_info=exc)
    elif status_code == 503:
        LOGGER.warning("Robot Service operation unavailable", exc_info=exc)
    if status_code in (400, 409, 422):
        message = _redact_sensitive_text(str(exc) or type(exc).__name__)
    elif status_code == 503:
        message = "Robot service is temporarily unavailable."
    else:
        message = "Internal server error."
    error_type = type(exc).__name__ if status_code != 500 else "InternalServerError"
    rejection_reason = _planning_rejection_reason(exc) if status_code == 422 else None
    return _error_payload(
        status_code,
        error_type,
        message,
        rejection_reason=rejection_reason,
    )


def _exception_status(exc: Exception) -> int:
    base_names = {item.__name__ for item in type(exc).__mro__}
    if isinstance(exc, RobotServiceStateError):
        return 409
    if isinstance(exc, RobotServiceCapabilityError):
        return 503 if exc.__cause__ is not None else 409
    if base_names & _CONFLICT_ERROR_BASE_NAMES:
        return 409
    if base_names & _PLANNING_ERROR_BASE_NAMES:
        return 422
    if isinstance(exc, (TypeError, ValueError)):
        return 400
    if isinstance(exc, (RobotServiceError, RuntimeError, OSError, TimeoutError)):
        return 503
    if type(exc).__module__.partition(".")[0] in {
        "drivers",
        "motion",
        "robot",
        "vision",
    }:
        return 503
    return 500


def _error_payload(
    status_code: int,
    error_type: str,
    message: str,
    *,
    rejection_reason: str | None = None,
) -> JSONResponse:
    error = {"type": error_type, "message": message}
    if rejection_reason is not None:
        error["rejection_reason"] = rejection_reason
    return JSONResponse(
        status_code=status_code,
        content={"error": error},
    )


def _planning_rejection_reason(exc: Exception) -> str:
    stage = getattr(exc, "stage", None)
    if isinstance(stage, str) and stage:
        return stage
    if "TargetOutsideTrayWorkspace" in {
        item.__name__ for item in type(exc).__mro__
    }:
        return "outside_tray_workspace"
    return re.sub(r"(?<!^)(?=[A-Z])", "_", type(exc).__name__).lower()


def _jsonable(value: object) -> object:
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, str):
        return _redact_sensitive_text(value)
    if isinstance(value, Enum):
        return _jsonable(value.value)
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        number = float(value)
        if not math.isfinite(number):
            raise _ResponseEncodingError("non-finite number in Service response")
        return number
    if hasattr(value, "translation_mm") and hasattr(value, "rpy_deg"):
        return {
            "translation_mm": _jsonable(tuple(value.translation_mm)),
            "rotation_rpy_deg": _jsonable(tuple(value.rpy_deg)),
        }
    if is_dataclass(value):
        return {
            item.name: _jsonable(getattr(value, item.name))
            for item in fields(value)
            if item.name not in {"tool_T_camera", "camera_T_target"}
        }
    if isinstance(value, Mapping):
        return {_json_key(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_jsonable(item) for item in value]
    raise _ResponseEncodingError(
        f"Service response type {type(value).__name__} is not public JSON data"
    )


def _json_key(value: object) -> str:
    if isinstance(value, Enum):
        return str(value.value)
    if isinstance(value, str):
        return _redact_sensitive_text(value)
    if isinstance(value, (bool, Integral)):
        return str(value)
    raise _ResponseEncodingError(
        f"Service response key type {type(value).__name__} is not public JSON data"
    )


def _redact_sensitive_text(value: str) -> str:
    redacted = _SENSITIVE_POSIX_PATH.sub("<redacted-path>", value)
    redacted = _SENSITIVE_WINDOWS_PATH.sub("<redacted-path>", redacted)
    return _SERIAL_PORT_NAME.sub("<redacted-port>", redacted)


__all__ = [
    "AxisAbsoluteMoveRequest",
    "AxisRelativeMoveRequest",
    "BaseTargetRequest",
    "DEFAULT_CORS_ORIGINS",
    "SuctionRequest",
    "create_robot_web_app",
]
