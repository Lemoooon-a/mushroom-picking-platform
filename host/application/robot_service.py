"""机器人进程级唯一入口：生命周期、状态、视觉与抓取工作流编排。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time

from application.controller import MushroomRobotController
from application.execution_record import ExecutionRecorder, NullExecutionRecorder
from application.grasp_profile import GraspProfile, GraspYawMode
from application.motion_target import BaseToolTarget
from application.ports import _AxisMotionPort
from application.pick_planner import PickPlan
from application.pick_workflow import (
    NoVisionTarget,
    PickOutcome,
    PickResult,
    VisionPickWorkflow,
)
from application.runtime_state import RobotServiceMode, RobotServiceState
from application.scan_pick import (
    ScanAndPickResult,
    ScanPickProfile,
    ScanPositionResult,
)
from calibration.hand_eye import HandEyeCalibrationStatus
from geometry.rigid_transform import RigidTransform
from kinematics.frame_chain import RobotAxisState
from motion.unified_protocol import AxisName, AxisState
from motion.unified_protocol import (
    AxisDescriptor,
    AxisTarget,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    RelativeAxisTarget,
)
from motion.unified_controller import UnifiedMotionError
from motion.authorization import MotionAuthorizationError
from vision.observation import VisionTargetObservation


_CAMERA_COLOR_OPTICAL_FRAME = "camera_color_optical_frame"
_ALL_AXIS_NAMES = tuple(AxisName)
_EXPECTED_POSITION_UNITS = {
    AxisName.SLIDE: "mm",
    AxisName.Z: "mm",
    AxisName.SHOULDER: "deg",
    AxisName.ELBOW: "deg",
    AxisName.ROTATION: "deg",
}


class RobotServiceError(RuntimeError):
    pass


class RobotServiceStateError(RobotServiceError):
    pass


class RobotServiceCapabilityError(RobotServiceError):
    pass


@dataclass
class _ActiveOperation:
    operation_id: int
    kind: str
    cancellation_requested: bool = False
    initial_state: RobotServiceState = RobotServiceState.READY


@dataclass(frozen=True)
class RobotServiceCapabilities:
    base_frame_motion: bool
    tray_workspace_gate: bool
    offset_planning: bool
    robot_motion_envelope: bool
    joint_holding: bool
    suction_command: bool
    vision_gateway: str
    vision_target_observation: bool
    hand_eye_calibration: HandEyeCalibrationStatus
    vision_target_resolution: bool
    pick_planning: bool
    pick_execution: bool
    physical_pick_verification: bool
    axis_listing: bool = False
    axis_state_query: bool = False
    axis_absolute_motion: bool = False
    axis_relative_motion: bool = False


@dataclass(frozen=True)
class RobotServiceStatus:
    state: RobotServiceState
    mode: RobotServiceMode
    capabilities: RobotServiceCapabilities
    backend_status: object | None
    fault: str | None


@dataclass(frozen=True)
class MotionResult:
    executed: bool
    plan: object
    message: str


@dataclass(frozen=True)
class CurrentTcpPose:
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float
    frame_id: str = "base"


@dataclass(frozen=True)
class ResolvedCameraPoint:
    camera_point_mm: tuple[float, float, float]
    base_point_mm: tuple[float, float, float]
    frame_id: str
    tool_camera_source: str
    tool_camera_validated: bool
    raw_base_point_mm: tuple[float, float, float] | None = None
    target_compensation_base_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_compensation_camera_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if self.raw_base_point_mm is None:
            object.__setattr__(self, "raw_base_point_mm", self.base_point_mm)

    @property
    def transform_status(self) -> HandEyeCalibrationStatus:
        return (
            HandEyeCalibrationStatus.VALIDATED
            if self.tool_camera_validated
            else HandEyeCalibrationStatus.PROVISIONAL
        )


class MushroomRobotService:
    def __init__(
        self,
        *,
        controller: MushroomRobotController,
        workflow: VisionPickWorkflow | None,
        mode: RobotServiceMode,
        grasp_profile: GraspProfile | None = None,
        scan_pick_profile: ScanPickProfile | None = None,
        axis_motion: _AxisMotionPort | None = None,
        recorder: ExecutionRecorder | None = None,
        activate_controller_on_startup: bool | None = None,
        vision_gateway_description: str = "unavailable",
        allow_dry_run_state_advance: bool = False,
    ) -> None:
        if not isinstance(controller, MushroomRobotController):
            raise TypeError("controller must be a MushroomRobotController")
        if workflow is not None and not isinstance(workflow, VisionPickWorkflow):
            raise TypeError("workflow must be VisionPickWorkflow or None")
        if not isinstance(mode, RobotServiceMode):
            raise TypeError("mode must be a RobotServiceMode")
        if grasp_profile is not None and not isinstance(grasp_profile, GraspProfile):
            raise TypeError("grasp_profile must be a GraspProfile or None")
        if scan_pick_profile is not None and not isinstance(
            scan_pick_profile, ScanPickProfile
        ):
            raise TypeError("scan_pick_profile must be a ScanPickProfile or None")
        self._axis_motion = axis_motion
        self._controller = controller
        self._workflow = workflow
        self.mode = mode
        self.grasp_profile = grasp_profile
        self.scan_pick_profile = scan_pick_profile
        self.recorder = recorder or NullExecutionRecorder()
        self.activate_controller_on_startup = (
            mode is not RobotServiceMode.READ_ONLY
            if activate_controller_on_startup is None
            else bool(activate_controller_on_startup)
        )
        self.vision_gateway_description = vision_gateway_description
        self._allow_dry_run_state_advance = bool(allow_dry_run_state_advance)
        self.state = RobotServiceState.CREATED
        self.fault: str | None = None
        self._started_controller = False
        self._state_lock = threading.RLock()
        self._next_operation_id = 1
        self._active_operation: _ActiveOperation | None = None
        self._shutdown_requested = False

    @property
    def capabilities(self) -> RobotServiceCapabilities:
        base = self._controller.capabilities
        vision_observation = self._workflow is not None
        pick_planning = bool(base.vision_target_resolution and self.grasp_profile is not None and vision_observation)
        with self._state_lock:
            state = self.state
            started_controller = self._started_controller
        axis_listing = callable(getattr(self._axis_motion, "list_axes", None))
        axis_state_query = callable(
            getattr(self._axis_motion, "get_axis_states", None)
        ) and (
            self.mode is RobotServiceMode.READ_ONLY
            or (
                self.mode in (RobotServiceMode.DRY_RUN, RobotServiceMode.EXECUTE)
                and started_controller
            )
        )
        axis_motion = (
            self.mode is RobotServiceMode.EXECUTE
            and state is RobotServiceState.READY
            and started_controller
            and callable(getattr(self._axis_motion, "submit_absolute", None))
        )
        return RobotServiceCapabilities(
            base_frame_motion=base.base_frame_motion,
            tray_workspace_gate=True,
            offset_planning=True,
            robot_motion_envelope=True,
            joint_holding=base.rotary_joint_enable_control,
            suction_command=base.suction_control,
            vision_gateway=self.vision_gateway_description,
            vision_target_observation=vision_observation,
            hand_eye_calibration=base.hand_eye_calibration,
            vision_target_resolution=base.vision_target_resolution,
            pick_planning=pick_planning,
            pick_execution=pick_planning and self.mode is RobotServiceMode.EXECUTE,
            physical_pick_verification=False,
            axis_listing=axis_listing,
            axis_state_query=axis_state_query,
            axis_absolute_motion=axis_motion,
            axis_relative_motion=axis_motion and callable(
                getattr(self._axis_motion, "submit_relative", None)
            ),
        )

    @property
    def tray_workspace(self) -> object:
        return self._controller.tray_workspace

    def startup(self) -> None:
        token = self._begin_write_operation(
            kind="startup",
            allowed_states=(RobotServiceState.CREATED, RobotServiceState.SHUTDOWN),
            active_state=RobotServiceState.STARTING,
            reset_shutdown_request=True,
        )
        try:
            if self.activate_controller_on_startup:
                self._controller.startup()
                with self._state_lock:
                    if self._active_operation is token:
                        self._started_controller = True
            finished = self._finish_operation(
                token,
                final_state=RobotServiceState.READY,
            )
            if not finished:
                try:
                    self._controller.shutdown()
                finally:
                    with self._state_lock:
                        self._started_controller = False
                self._record("startup", final_status="cancelled")
                return
            self._record("startup", final_status="ready")
        except Exception as exc:
            if not self._operation_is_current(token):
                self._record("startup", final_status="cancelled", error=str(exc))
                raise
            if getattr(exc, "stop_report", None) is None:
                self._best_effort_stop()
            compensation_errors: list[str] = []
            try:
                self._controller.shutdown()
            except Exception as close_exc:
                compensation_errors.append(f"shutdown compensation failed: {close_exc}")
            with self._state_lock:
                if self._active_operation is token:
                    self._started_controller = False
            fault = f"startup: {exc}"
            if compensation_errors:
                fault += "; " + "; ".join(compensation_errors)
            self._finish_operation(
                token,
                final_state=RobotServiceState.FAULT,
                fault=fault,
            )
            self._record("startup", final_status="fault", error=fault)
            raise

    def shutdown(self) -> None:
        with self._state_lock:
            if (
                self.state is RobotServiceState.SHUTDOWN
                and self._active_operation is None
            ):
                self._shutdown_requested = True
                return
            self._shutdown_requested = True
            token = self._active_operation
            if token is not None:
                token.cancellation_requested = True
                self._active_operation = None
            started_controller = self._started_controller
        stop_error: Exception | None = None
        close_error: Exception | None = None
        if token is not None and self.mode is RobotServiceMode.EXECUTE:
            try:
                self._controller.stop()
            except Exception as exc:
                stop_error = exc
        if started_controller or token is not None:
            try:
                self._controller.shutdown()
            except Exception as exc:
                close_error = exc
        with self._state_lock:
            self._started_controller = False
            self.state = RobotServiceState.SHUTDOWN
            errors = tuple(
                message
                for message in (
                    f"stop failed: {stop_error}" if stop_error else None,
                    f"close failed: {close_error}" if close_error else None,
                )
                if message is not None
            )
            if errors:
                self.fault = "shutdown: " + "; ".join(errors)
        self._record(
            "shutdown",
            final_status="shutdown" if close_error is None else "fault",
            error=str(close_error) if close_error is not None else None,
        )
        if close_error is not None:
            raise close_error

    def status(self) -> RobotServiceStatus:
        with self._state_lock:
            state = self.state
            fault = self.fault
            started_controller = self._started_controller
        backend_status = None
        if started_controller and state in (
            RobotServiceState.READY,
            RobotServiceState.DISABLED,
        ):
            try:
                backend_status = self._controller.get_status()
            except Exception as exc:
                backend_status = f"unavailable: {exc}"
        return RobotServiceStatus(
            state,
            self.mode,
            self.capabilities,
            backend_status,
            fault,
        )

    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        method = getattr(self._axis_motion, "list_axes", None)
        if not callable(method):
            raise RobotServiceCapabilityError("axis listing is unavailable")
        descriptors = method()
        if not isinstance(descriptors, tuple) or not all(
            isinstance(item, AxisDescriptor) for item in descriptors
        ):
            raise RobotServiceError("axis motion port returned invalid descriptors")
        return descriptors

    def get_axis_state(self, axis: AxisName) -> AxisState:
        if not isinstance(axis, AxisName):
            raise ValueError("axis must be an AxisName")
        self._require_axis_query_runtime()
        method = getattr(self._axis_motion, "get_state", None)
        if not callable(method):
            raise RobotServiceCapabilityError("axis state query is unavailable")
        try:
            state = method(axis)
        except Exception as exc:
            raise RobotServiceCapabilityError(
                f"axis state query is unavailable: {exc}"
            ) from exc
        if not isinstance(state, AxisState):
            raise RobotServiceError("axis motion port returned an invalid state")
        return state

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]:
        if axes is not None and (
            not isinstance(axes, tuple)
            or not all(isinstance(axis, AxisName) for axis in axes)
        ):
            raise ValueError("axes must be a tuple of AxisName values or None")
        self._require_axis_query_runtime()
        method = getattr(self._axis_motion, "get_axis_states", None)
        if not callable(method):
            raise RobotServiceCapabilityError("axis state query is unavailable")
        try:
            states = method(axes)
        except Exception as exc:
            raise RobotServiceCapabilityError(
                f"axis state query is unavailable: {exc}"
            ) from exc
        if not isinstance(states, tuple) or not all(
            isinstance(state, AxisState) for state in states
        ):
            raise RobotServiceError("axis motion port returned invalid states")
        return states

    def _require_axis_query_runtime(self) -> None:
        with self._state_lock:
            started_controller = self._started_controller
        if (
            self.mode in (RobotServiceMode.DRY_RUN, RobotServiceMode.EXECUTE)
            and not started_controller
        ):
            raise RobotServiceCapabilityError(
                "axis state query requires a started Robot Service runtime"
            )

    def move_axis_absolute(
        self,
        axis: AxisName,
        position: float,
        *,
        velocity: float | None = None,
        acceleration: float | None = None,
        timeout_s: float | None = None,
    ) -> MotionCommandResult:
        try:
            target = AxisTarget(axis, position, velocity, acceleration)
        except Exception as exc:
            self._record_invalid_axis_request(
                "absolute", axis, position, velocity, acceleration, timeout_s, exc
            )
            raise
        return self._move_axis(
            command_kind="absolute",
            target=target,
            timeout_s=timeout_s,
            requested_delta=None,
        )

    def move_axis_relative(
        self,
        axis: AxisName,
        delta: float,
        *,
        velocity: float | None = None,
        acceleration: float | None = None,
        timeout_s: float | None = None,
    ) -> MotionCommandResult:
        try:
            target = RelativeAxisTarget(axis, delta, velocity, acceleration)
        except Exception as exc:
            self._record_invalid_axis_request(
                "relative", axis, delta, velocity, acceleration, timeout_s, exc
            )
            raise
        return self._move_axis(
            command_kind="relative",
            target=target,
            timeout_s=timeout_s,
            requested_delta=target.delta,
        )

    def _move_axis(
        self,
        *,
        command_kind: str,
        target: AxisTarget | RelativeAxisTarget,
        timeout_s: float | None,
        requested_delta: float | None,
    ) -> MotionCommandResult:
        submit = getattr(
            self._axis_motion,
            "submit_absolute" if command_kind == "absolute" else "submit_relative",
            None,
        )
        wait = getattr(self._axis_motion, "wait", None)
        token: _ActiveOperation | None = None
        try:
            self._require_execute_mode(f"raw axis {command_kind} motion")
            _finite_positive_optional("timeout_s", timeout_s)
            with self._state_lock:
                started_controller = self._started_controller
            if not started_controller or not callable(submit) or not callable(wait):
                raise RobotServiceCapabilityError(
                    f"raw axis {command_kind} motion is unavailable"
                )
            token = self._begin_write_operation(
                kind=f"raw axis {command_kind} motion",
                allowed_states=(RobotServiceState.READY,),
                active_state=RobotServiceState.EXECUTING,
            )
        except Exception as exc:
            self._record_axis_motion(
                command_kind,
                target,
                timeout_s,
                submitted=False,
                no_op=False,
                final_status="rejected",
                error=str(exc),
            )
            raise
        assert token is not None
        assert callable(submit) and callable(wait)
        submitted = False
        try:
            handle = submit(target)
            submitted = True
            result = wait(handle, timeout_s=timeout_s)
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if not submitted and _is_pre_submission_rejection(exc):
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.READY,
                )
                final_status = "rejected"
            else:
                if operation_current:
                    self._best_effort_stop_axis(target.axis)
                    self._finish_operation(
                        token,
                        final_state=RobotServiceState.FAULT,
                        fault=f"raw axis {command_kind} motion: {exc}",
                    )
                    final_status = "fault"
                else:
                    final_status = "cancelled"
            self._record_axis_motion(
                command_kind,
                target,
                timeout_s,
                submitted=submitted,
                no_op=False,
                final_status=final_status,
                error=str(exc),
            )
            raise

        no_op = "no motion submitted" in result.message
        if result.status is MotionCommandStatus.ARRIVED:
            self._finish_operation(
                token,
                final_state=RobotServiceState.READY,
            )
        else:
            if self._operation_is_current(token):
                self._best_effort_stop_axis(target.axis)
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=(
                        f"raw axis {command_kind} motion: {result.status.value}: "
                        f"{result.message}"
                    ),
                )
        self._record_axis_motion(
            command_kind,
            target,
            timeout_s,
            submitted=not no_op,
            no_op=no_op,
            final_status=result.status.value,
            terminal_outcome=result,
            start_position=(
                result.target_position - requested_delta
                if requested_delta is not None
                else None
            ),
            resolved_absolute_target=result.target_position,
        )
        return result

    def _record_axis_motion(
        self,
        command_kind: str,
        target: AxisTarget | RelativeAxisTarget,
        timeout_s: float | None,
        **fields: object,
    ) -> None:
        self._record(
            "raw-axis-motion",
            command_kind=command_kind,
            axis=target.axis,
            requested_absolute_target=(
                target.position if isinstance(target, AxisTarget) else None
            ),
            requested_delta=(
                target.delta if isinstance(target, RelativeAxisTarget) else None
            ),
            velocity=target.velocity,
            acceleration=target.acceleration,
            timeout_s=timeout_s,
            **fields,
        )

    def _record_invalid_axis_request(
        self,
        command_kind: str,
        axis: object,
        requested_value: object,
        velocity: object,
        acceleration: object,
        timeout_s: object,
        exc: Exception,
    ) -> None:
        self._record(
            "raw-axis-motion",
            command_kind=command_kind,
            axis=axis,
            requested_value=requested_value,
            velocity=velocity,
            acceleration=acceleration,
            timeout_s=timeout_s,
            submitted=False,
            no_op=False,
            final_status="rejected",
            error=str(exc),
        )

    def _best_effort_stop_axis(self, axis: AxisName) -> None:
        method = getattr(self._axis_motion, "stop", None)
        if callable(method):
            try:
                method(axis)
            except Exception:
                pass

    def resolve_camera_point(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        *,
        frame_id: str = _CAMERA_COLOR_OPTICAL_FRAME,
        capture_axis_state: RobotAxisState | None = None,
    ) -> ResolvedCameraPoint:
        """把 Camera 点按当前姿态或已确认的拍照快照转换到 Base frame。"""

        self._require_ready("resolve-camera-point")
        camera_point = (
            _finite_coordinate("x_mm", x_mm),
            _finite_coordinate("y_mm", y_mm),
            _finite_coordinate("z_mm", z_mm),
        )
        if camera_point[2] <= 0.0:
            raise ValueError("z_mm must be greater than zero")
        if frame_id != _CAMERA_COLOR_OPTICAL_FRAME:
            raise ValueError(
                f"frame_id must be {_CAMERA_COLOR_OPTICAL_FRAME!r}"
            )

        resolver = self._controller.target_resolver
        calibration = None if resolver is None else resolver.hand_eye_calibration
        if resolver is None or calibration is None:
            raise RobotServiceCapabilityError("tool_T_camera is not configured.")

        if capture_axis_state is None:
            axis_state = self._read_current_axis_state()
        elif isinstance(capture_axis_state, RobotAxisState):
            axis_state = capture_axis_state
        else:
            raise TypeError("capture_axis_state must be a RobotAxisState or None")
        base_T_tool = resolver.pose_provider.forward_kinematics_base(axis_state)
        if not isinstance(base_T_tool, RigidTransform):
            raise RobotServiceError(
                "current pose provider returned a non-RigidTransform base_T_tool"
            )
        raw_base_point = tuple(
            float(value)
            for value in (base_T_tool @ calibration.tool_T_camera).transform_point(
                camera_point
            )
        )
        compensated_camera_point = calibration.compensate_camera_point(camera_point)
        camera_compensated_base_point = tuple(
            float(value)
            for value in (base_T_tool @ calibration.tool_T_camera).transform_point(
                compensated_camera_point
            )
        )
        base_point = calibration.compensate_base_point(
            camera_compensated_base_point
        )
        return ResolvedCameraPoint(
            camera_point_mm=camera_point,
            base_point_mm=base_point,
            frame_id=frame_id,
            tool_camera_source=calibration.source,
            tool_camera_validated=calibration.validated,
            raw_base_point_mm=raw_base_point,
            target_compensation_base_mm=calibration.target_compensation_base_mm,
            target_compensation_camera_mm=(
                calibration.target_compensation_camera_mm
            ),
        )

    def get_current_tcp_pose(self) -> CurrentTcpPose:
        """Read the current five-axis state and resolve Base-frame TCP pose."""

        self._require_ready("current TCP pose")
        resolver = self._controller.target_resolver
        if resolver is None:
            raise RobotServiceCapabilityError(
                "Current Base-frame TCP pose provider is unavailable."
            )
        axis_state = self._read_current_axis_state()
        base_T_tool = resolver.pose_provider.forward_kinematics_base(axis_state)
        if not isinstance(base_T_tool, RigidTransform):
            raise RobotServiceError(
                "current pose provider returned a non-RigidTransform base_T_tool"
            )
        x_mm, y_mm, z_mm = (
            float(value) for value in base_T_tool.translation_mm
        )
        return CurrentTcpPose(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            yaw_deg=float(base_T_tool.yaw_deg),
        )

    def plan_base_target(self, target: BaseToolTarget) -> object:
        self._require_not_read_only("plan")
        token = self._begin_write_operation(
            kind="plan",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.PLANNING,
        )
        try:
            plan = self._controller.plan_base_target(target)
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("plan", input_target=target, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("plan", input_target=target, final_status="rejected", error=str(exc))
            raise

    def move_base_target(self, target: BaseToolTarget) -> MotionResult:
        self._require_not_read_only("move")
        token = self._begin_write_operation(
            kind="move",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.PLANNING,
        )
        try:
            plan = self._controller.plan_base_target(target)
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("plan", input_target=target, final_status="rejected", error=str(exc))
            raise
        self._record("plan", input_target=target, selected_plan=plan, final_status="ready")
        if self.mode is not RobotServiceMode.EXECUTE:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            return MotionResult(False, plan, "Dry-run plan complete; no motion command was submitted.")
        if not self._set_operation_state(token, RobotServiceState.EXECUTING):
            raise RobotServiceStateError("move was cancelled before execution")
        try:
            self._controller.execute_base_plan(plan)
            self._finish_operation(token, final_state=RobotServiceState.READY)
            result = MotionResult(True, plan, "Base motion completed.")
            self._record("move", input_target=target, selected_plan=plan, final_status="ready")
            return result
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if operation_current:
                if getattr(exc, "stop_report", None) is None:
                    self._best_effort_stop()
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=f"move: {exc}",
                )
            self._record(
                "move",
                final_status="fault" if operation_current else "cancelled",
                error=str(exc),
            )
            raise

    def request_observation(self) -> VisionTargetObservation:
        self._require_not_read_only("observe")
        workflow = self._require_workflow()
        token = self._begin_write_operation(
            kind="observe",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.OBSERVING,
        )
        try:
            observation = workflow.request_observation()
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("observe", request_id=observation.request_id, final_status="ready")
            return observation
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("observe", final_status="rejected", error=str(exc))
            raise

    def plan_observation(self, observation: VisionTargetObservation, grasp_profile: GraspProfile | None = None) -> PickPlan:
        self._require_not_read_only("plan-observation")
        if not self._controller.capabilities.vision_target_resolution:
            raise RobotServiceCapabilityError(
                "Hand-eye calibration is missing or not validated."
            )
        profile = self._require_profile(grasp_profile)
        workflow = self._require_workflow()
        token = self._begin_write_operation(
            kind="plan-observation",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.PLANNING,
        )
        try:
            plan = workflow.plan_observation(observation, profile)
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("plan-observation", request_id=observation.request_id, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("plan-observation", request_id=observation.request_id, final_status="rejected", error=str(exc))
            raise

    def execute_pick_plan(self, plan: PickPlan) -> PickResult:
        workflow = self._require_workflow()
        execute = self.mode is RobotServiceMode.EXECUTE
        token = self._begin_write_operation(
            kind="pick",
            allowed_states=(RobotServiceState.READY,),
            active_state=(
                RobotServiceState.EXECUTING
                if execute
                else RobotServiceState.PLANNING
            ),
        )
        try:
            result = workflow.execute_pick_plan(plan, execute=execute)
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if execute and operation_current:
                self._best_effort_stop()
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=f"pick: {exc}",
                )
            elif operation_current:
                self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record(
                "pick",
                final_status=(
                    "fault" if execute else "rejected"
                ) if operation_current else "cancelled",
                error=str(exc),
            )
            raise
        if result.outcome is PickOutcome.FAILED:
            self._finish_operation(
                token,
                final_state=RobotServiceState.FAULT,
                fault=result.message,
            )
        else:
            self._finish_operation(token, final_state=RobotServiceState.READY)
        self._record("pick", request_id=plan.observation.request_id, selected_plan=plan, final_status=self.state.value, stage_result=result)
        return result

    def pick(self, grasp_profile: GraspProfile | None = None) -> PickResult:
        self._require_not_read_only("pick")
        if not self._controller.capabilities.vision_target_resolution:
            raise RobotServiceCapabilityError(
                "Hand-eye calibration is missing or not validated."
            )
        profile = self._require_profile(grasp_profile)
        workflow = self._require_workflow()
        token = self._begin_write_operation(
            kind="pick",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.OBSERVING,
        )
        try:
            observation = workflow.request_observation()
            if not self._set_operation_state(token, RobotServiceState.PLANNING):
                raise RobotServiceStateError("pick was cancelled before planning")
            plan = workflow.plan_observation(observation, profile)
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record("pick", final_status="rejected", error=str(exc))
            raise
        execute = self.mode is RobotServiceMode.EXECUTE
        next_state = (
            RobotServiceState.EXECUTING if execute else RobotServiceState.PLANNING
        )
        if not self._set_operation_state(token, next_state):
            raise RobotServiceStateError("pick was cancelled before execution")
        try:
            result = workflow.execute_pick_plan(plan, execute=execute)
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if execute and operation_current:
                self._best_effort_stop()
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=f"pick: {exc}",
                )
            elif operation_current:
                self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record(
                "pick",
                final_status=(
                    "fault" if execute else "rejected"
                ) if operation_current else "cancelled",
                error=str(exc),
            )
            raise
        final_state = (
            RobotServiceState.FAULT
            if result.outcome is PickOutcome.FAILED
            else RobotServiceState.READY
        )
        self._finish_operation(
            token,
            final_state=final_state,
            fault=result.message if final_state is RobotServiceState.FAULT else None,
        )
        self._record(
            "pick",
            request_id=observation.request_id,
            selected_plan=plan,
            final_status=final_state.value,
            stage_result=result,
        )
        return result

    def move_to_scan_position(self, scan_index: int) -> MotionResult:
        """Move to one validated scan pose through the existing Base motion API."""

        scan_pose = self._scan_pose(scan_index)
        return self.move_base_target(scan_pose)

    def pick_one_at_scan_position(self, scan_index: int) -> ScanAndPickResult:
        """Pick, place, and return for one validated scan position."""

        operation_kind = "scan-pick-one"
        workflow, grasp_profile, scan_profile = self._scan_pick_context(
            operation_kind
        )
        scan_pose = self._scan_pose(scan_index, scan_profile=scan_profile)
        token = self._begin_write_operation(
            kind=operation_kind,
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.PLANNING,
        )
        self._plan_and_execute_scan_motion(
            token,
            scan_pose,
            operation_kind=operation_kind,
        )
        position_result = self._pick_one_from_scan_position(
            token,
            workflow=workflow,
            grasp_profile=grasp_profile,
            scan_profile=scan_profile,
            scan_index=scan_index,
            scan_pose=scan_pose,
            operation_kind=operation_kind,
        )
        result = ScanAndPickResult(
            "completed",
            (position_result,),
            position_result.picked_count,
        )
        self._finish_operation(token, final_state=RobotServiceState.READY)
        self._record(
            operation_kind,
            final_status="completed",
            stage_result=result,
        )
        return result

    def scan_and_pick(self) -> ScanAndPickResult:
        """以单个 Service operation 完成固定八区域扫描、抓取和放置。"""

        operation_kind = "scan-pick"
        workflow, grasp_profile, scan_profile = self._scan_pick_context(
            operation_kind
        )

        token = self._begin_write_operation(
            kind=operation_kind,
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.PLANNING,
        )
        visited: list[ScanPositionResult] = []
        total_picked = 0

        for scan_index, scan_pose in enumerate(scan_profile.scan_poses, start=1):
            self._plan_and_execute_scan_motion(
                token,
                scan_pose,
                operation_kind=operation_kind,
            )

            detected_count = 0
            picked_count = 0
            while picked_count < scan_profile.max_picks_per_scan_pose:
                attempt = self._pick_one_from_scan_position(
                    token,
                    workflow=workflow,
                    grasp_profile=grasp_profile,
                    scan_profile=scan_profile,
                    scan_index=scan_index,
                    scan_pose=scan_pose,
                    operation_kind=operation_kind,
                )
                detected_count += attempt.detected_count
                picked_count += attempt.picked_count
                total_picked += attempt.picked_count
                if attempt.picked_count == 0:
                    visited.append(
                        ScanPositionResult(
                            scan_index,
                            detected_count,
                            picked_count,
                            attempt.final_reason,
                        )
                    )
                    break
            else:
                visited.append(
                    ScanPositionResult(
                        scan_index,
                        detected_count,
                        picked_count,
                        "max_picks_per_scan_pose_reached",
                    )
                )
                result = ScanAndPickResult(
                    "stopped_max_picks_per_scan_pose",
                    tuple(visited),
                    total_picked,
                )
                self._finish_operation(token, final_state=RobotServiceState.READY)
                self._record(
                    "scan-pick",
                    final_status=result.result,
                    stage_result=result,
                )
                return result

        result = ScanAndPickResult("completed", tuple(visited), total_picked)
        self._finish_operation(token, final_state=RobotServiceState.READY)
        self._record("scan-pick", final_status="completed", stage_result=result)
        return result

    def return_to_startup(self) -> object:
        self._require_execute_mode("return")
        token = self._begin_write_operation(
            kind="return",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.EXECUTING,
        )
        try:
            result = self._controller.return_to_startup()
            self._finish_operation(token, final_state=RobotServiceState.READY)
            return result
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if operation_current:
                self._best_effort_stop()
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=f"return: {exc}",
                )
            self._record(
                "return",
                final_status="fault" if operation_current else "cancelled",
                error=str(exc),
            )
            raise

    def stop(self) -> None:
        with self._state_lock:
            token = self._active_operation
            if token is not None:
                token.cancellation_requested = True
                self._active_operation = None
            elif self.state is RobotServiceState.EXECUTING:
                token = _ActiveOperation(
                    0,
                    "orphan executing operation",
                    cancellation_requested=True,
                    initial_state=RobotServiceState.READY,
                )
            started_controller = self._started_controller
            shutdown_requested = self._shutdown_requested
        stop_error: Exception | None = None
        if self.mode is RobotServiceMode.EXECUTE and started_controller:
            try:
                self._controller.stop()
            except Exception as exc:
                stop_error = exc
        state_reader = getattr(self._axis_motion, "get_axis_states", None)
        if (
            token is not None
            and self.mode is RobotServiceMode.EXECUTE
            and started_controller
            and callable(state_reader)
            and stop_error is None
        ):
            try:
                states = state_reader(_ALL_AXIS_NAMES)
                valid_stop = (
                    isinstance(states, tuple)
                    and len(states) == len(_ALL_AXIS_NAMES)
                    and all(
                        isinstance(item, AxisState)
                        and item.busy is False
                        and not item.faulted
                        and item.position_valid
                        for item in states
                    )
                )
            except Exception as exc:
                valid_stop = False
                verification_error = f"stop state verification: {exc}"
            else:
                verification_error = None
        elif token is not None and self.mode is RobotServiceMode.EXECUTE:
            valid_stop = False
            verification_error = (
                f"stop: {stop_error}"
                if stop_error is not None
                else "stop did not confirm valid stationary axes"
            )
        else:
            valid_stop = True
            verification_error = None
        stopped_state = RobotServiceState.READY
        if token is not None and valid_stop:
            if token.initial_state is RobotServiceState.DISABLED:
                if self._rotary_holding_snapshot() is False:
                    stopped_state = RobotServiceState.DISABLED
                else:
                    valid_stop = False
                    verification_error = (
                        "cancelled enable did not confirm disabled joint holding"
                    )
            elif token.kind == "joints disable":
                if self._rotary_holding_snapshot() is not True:
                    valid_stop = False
                    verification_error = (
                        "cancelled disable did not confirm enabled joint holding"
                    )
        with self._state_lock:
            if stop_error is not None and not self._shutdown_requested:
                self.state = RobotServiceState.FAULT
                self.fault = f"stop: {stop_error}"
            elif token is not None and not self._shutdown_requested:
                if valid_stop:
                    self.state = stopped_state
                    self.fault = None
                else:
                    self.state = RobotServiceState.FAULT
                    self.fault = (
                        verification_error
                        or "stop did not confirm valid stationary axes"
                    )
            final_state = self.state
        self._record("stop", final_status=final_state.value)
        if stop_error is not None and not shutdown_requested:
            raise stop_error

    def enable_joints(self) -> object:
        self._require_execute_mode("joints enable")
        token = self._begin_write_operation(
            kind="joints enable",
            allowed_states=(RobotServiceState.DISABLED, RobotServiceState.READY),
            active_state=RobotServiceState.EXECUTING,
        )
        try:
            result = self._controller.enable_joints()
            if not self._operation_is_current(token):
                self._record("joints enable", final_status="cancelled")
                return result
            self._require_enabled_stationary_axes()
        except Exception as exc:
            if not self._operation_is_current(token):
                self._record("joints enable", final_status="cancelled", error=str(exc))
                raise
            holding = self._rotary_holding_snapshot()
            final_state = (
                RobotServiceState.DISABLED
                if holding is False
                else RobotServiceState.FAULT
            )
            self._finish_operation(
                token,
                final_state=final_state,
                fault=None if final_state is RobotServiceState.DISABLED else f"joints enable: {exc}",
            )
            self._record("joints enable", final_status=final_state.value, error=str(exc))
            raise
        self._finish_operation(token, final_state=RobotServiceState.READY)
        self._record("joints enable", final_status="ready")
        return result

    def disable_joints(self) -> object:
        self._require_execute_mode("joints disable")
        token = self._begin_write_operation(
            kind="joints disable",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.EXECUTING,
        )
        try:
            result = self._controller.disable_joints()
        except Exception as exc:
            if not self._operation_is_current(token):
                self._record("joints disable", final_status="cancelled", error=str(exc))
                raise
            holding = self._rotary_holding_snapshot()
            final_state = (
                RobotServiceState.READY
                if holding is True
                else RobotServiceState.DISABLED
                if holding is False
                else RobotServiceState.FAULT
            )
            self._finish_operation(
                token,
                final_state=final_state,
                fault=None if final_state is not RobotServiceState.FAULT else f"joints disable: {exc}",
            )
            self._record("joints disable", final_status=final_state.value, error=str(exc))
            raise
        self._finish_operation(token, final_state=RobotServiceState.DISABLED)
        self._record("joints disable", final_status="disabled")
        return result

    def suction(self, action: str) -> object:
        methods = {
            "grip": self._controller.suction_grip,
            "release": self._controller.suction_release,
            "idle": self._controller.suction_idle,
        }
        if action not in methods:
            raise ValueError("suction action must be grip, release, or idle")
        self._require_execute_mode(f"suction {action}")
        token = self._begin_write_operation(
            kind=f"suction {action}",
            allowed_states=(RobotServiceState.READY,),
            active_state=RobotServiceState.EXECUTING,
        )
        try:
            result = methods[action]()
        except Exception as exc:
            operation_current = self._operation_is_current(token)
            if operation_current:
                self._best_effort_stop()
                self._finish_operation(
                    token,
                    final_state=RobotServiceState.FAULT,
                    fault=f"suction {action}: {exc}",
                )
            self._record(
                f"suction {action}",
                final_status="fault" if operation_current else "cancelled",
                error=str(exc),
            )
            raise
        self._finish_operation(token, final_state=RobotServiceState.READY)
        self._record(f"suction {action}", final_status="ready")
        return result

    def _begin_write_operation(
        self,
        *,
        kind: str,
        allowed_states: tuple[RobotServiceState, ...],
        active_state: RobotServiceState,
        reset_shutdown_request: bool = False,
    ) -> _ActiveOperation:
        with self._state_lock:
            if reset_shutdown_request and self.state is RobotServiceState.SHUTDOWN:
                self._shutdown_requested = False
            if self._shutdown_requested:
                raise RobotServiceStateError(
                    f"{kind} is unavailable because shutdown was requested"
                )
            if self.state not in allowed_states:
                allowed = "/".join(state.name for state in allowed_states)
                raise RobotServiceStateError(
                    f"{kind} requires {allowed}, got {self.state.value}"
                )
            if self._active_operation is not None:
                raise RobotServiceStateError(
                    f"{kind} rejected because operation "
                    f"{self._active_operation.kind} is already active"
                )
            token = _ActiveOperation(
                self._next_operation_id,
                kind,
                initial_state=self.state,
            )
            self._next_operation_id += 1
            self._active_operation = token
            self.state = active_state
            return token

    def _set_operation_state(
        self,
        token: _ActiveOperation,
        state: RobotServiceState,
    ) -> bool:
        with self._state_lock:
            if (
                self._active_operation is not token
                or token.cancellation_requested
                or self._shutdown_requested
            ):
                return False
            self.state = state
            return True

    def _finish_operation(
        self,
        token: _ActiveOperation,
        *,
        final_state: RobotServiceState,
        fault: str | None = None,
    ) -> bool:
        with self._state_lock:
            if (
                self._active_operation is not token
                or token.cancellation_requested
                or self._shutdown_requested
            ):
                return False
            self.state = final_state
            self.fault = fault
            self._active_operation = None
            return True

    def _operation_is_current(self, token: _ActiveOperation) -> bool:
        with self._state_lock:
            return (
                self._active_operation is token
                and not token.cancellation_requested
                and not self._shutdown_requested
            )

    def _require_execute_mode(self, operation: str) -> None:
        if self.mode is not RobotServiceMode.EXECUTE:
            raise RobotServiceCapabilityError(
                f"{operation} requires execute mode and explicit motion authorization"
            )

    def _require_ready(self, operation: str) -> None:
        with self._state_lock:
            if self.state is not RobotServiceState.READY:
                raise RobotServiceStateError(
                    f"{operation} requires READY, got {self.state.value}"
                )

    def _require_not_read_only(self, operation: str) -> None:
        if self.mode is RobotServiceMode.READ_ONLY:
            raise RobotServiceCapabilityError(f"{operation} is unavailable in read-only mode")

    def _require_execute(self, operation: str) -> None:
        self._require_ready(operation)
        self._require_execute_mode(operation)

    def _require_workflow(self) -> VisionPickWorkflow:
        if self._workflow is None:
            raise RobotServiceCapabilityError("Vision gateway/workflow is unavailable.")
        return self._workflow

    def _require_profile(self, provided: GraspProfile | None) -> GraspProfile:
        profile = provided or self.grasp_profile
        if profile is None:
            raise RobotServiceCapabilityError("Grasp profile is missing or not validated.")
        return profile

    def _require_scan_pick_profile(self) -> ScanPickProfile:
        if self.scan_pick_profile is None:
            raise RobotServiceCapabilityError(
                "Scan-pick profile is missing or not validated."
            )
        return self.scan_pick_profile

    def _scan_pose(
        self,
        scan_index: int,
        *,
        scan_profile: ScanPickProfile | None = None,
    ) -> BaseToolTarget:
        if isinstance(scan_index, bool) or not isinstance(scan_index, int):
            raise TypeError("scan_index must be an integer from 1 through 8")
        if not 1 <= scan_index <= 8:
            raise ValueError("scan_index must be from 1 through 8")
        profile = scan_profile or self._require_scan_pick_profile()
        return profile.scan_poses[scan_index - 1]

    def _scan_pick_context(
        self,
        operation_kind: str,
    ) -> tuple[VisionPickWorkflow, GraspProfile, ScanPickProfile]:
        self._require_not_read_only(operation_kind)
        if not self._controller.capabilities.vision_target_resolution:
            raise RobotServiceCapabilityError(
                "Hand-eye calibration is missing or not validated."
            )
        workflow = self._require_workflow()
        grasp_profile = self._require_profile(None)
        scan_profile = self._require_scan_pick_profile()
        if (
            grasp_profile.yaw_mode is not GraspYawMode.FIXED
            or grasp_profile.fixed_yaw_deg != 0.0
        ):
            raise RobotServiceCapabilityError(
                f"{operation_kind} requires the validated grasp profile to use "
                "fixed yaw 0"
            )
        if (
            self.mode is RobotServiceMode.DRY_RUN
            and not self._allow_dry_run_state_advance
        ):
            raise RobotServiceCapabilityError(
                f"{operation_kind} dry-run requires the offline simulated-state "
                "backend"
            )
        return workflow, grasp_profile, scan_profile

    def _plan_and_execute_scan_motion(
        self,
        token: _ActiveOperation,
        scan_pose: BaseToolTarget,
        *,
        operation_kind: str,
    ) -> None:
        try:
            self._require_scan_operation_state(
                token,
                RobotServiceState.PLANNING,
                "scan motion planning",
                operation_kind=operation_kind,
            )
            scan_motion = self._controller.plan_base_target(scan_pose)
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record(
                operation_kind,
                final_status="rejected",
                error=str(exc),
            )
            raise
        try:
            self._execute_scan_motion(
                token,
                scan_motion,
                operation_kind=operation_kind,
            )
        except Exception as exc:
            self._fail_scan_task(token, exc, operation_kind=operation_kind)
            raise

    def _pick_one_from_scan_position(
        self,
        token: _ActiveOperation,
        *,
        workflow: VisionPickWorkflow,
        grasp_profile: GraspProfile,
        scan_profile: ScanPickProfile,
        scan_index: int,
        scan_pose: BaseToolTarget,
        operation_kind: str,
    ) -> ScanPositionResult:
        self._require_scan_operation_state(
            token,
            RobotServiceState.OBSERVING,
            "observation settling",
            operation_kind=operation_kind,
        )
        if scan_profile.scan_settle_time_s > 0.0:
            time.sleep(scan_profile.scan_settle_time_s)
        self._require_scan_operation_state(
            token,
            RobotServiceState.OBSERVING,
            "observation",
            operation_kind=operation_kind,
        )
        try:
            observation = workflow.request_observation()
        except NoVisionTarget:
            return ScanPositionResult(scan_index, 0, 0, "no_target")
        except Exception as exc:
            self._finish_operation(token, final_state=RobotServiceState.READY)
            self._record(
                operation_kind,
                final_status="rejected",
                error=str(exc),
            )
            raise

        self._require_scan_operation_state(
            token,
            RobotServiceState.PLANNING,
            "pick planning",
            operation_kind=operation_kind,
        )
        try:
            pick_plan = workflow.plan_observation(observation, grasp_profile)
        except Exception as exc:
            self._record(
                "scan-pick-target",
                request_id=observation.request_id,
                final_status="rejected",
                error=str(exc),
            )
            return ScanPositionResult(
                scan_index,
                1,
                0,
                f"target_rejected:{type(exc).__name__}",
            )

        try:
            self._require_scan_operation_state(
                token,
                (
                    RobotServiceState.EXECUTING
                    if self.mode is RobotServiceMode.EXECUTE
                    else RobotServiceState.PLANNING
                ),
                "pick execution",
                operation_kind=operation_kind,
            )
            pick_result = workflow.execute_pick_plan(pick_plan, execute=True)
            if pick_result.outcome is PickOutcome.FAILED:
                raise RobotServiceError(pick_result.message)

            self._require_scan_operation_state(
                token,
                RobotServiceState.PLANNING,
                "place planning",
                operation_kind=operation_kind,
            )
            place_pre = scan_profile.place_pre_pose
            place_motions = self._controller.plan_base_target_sequence(
                (place_pre, scan_profile.place_pose, place_pre),
                enforce_tray_workspace=(False, True, False),
            )
            self._require_scan_operation_state(
                token,
                (
                    RobotServiceState.EXECUTING
                    if self.mode is RobotServiceMode.EXECUTE
                    else RobotServiceState.PLANNING
                ),
                "place execution",
                operation_kind=operation_kind,
            )
            self._controller.execute_base_plan(place_motions[0])
            self._controller.execute_base_plan(place_motions[1])
            self._controller.suction_release()
            self._controller.execute_base_plan(place_motions[2])

            self._require_scan_operation_state(
                token,
                RobotServiceState.PLANNING,
                "scan return planning",
                operation_kind=operation_kind,
            )
            return_motion = self._controller.plan_base_target(scan_pose)
            self._execute_scan_motion(
                token,
                return_motion,
                operation_kind=operation_kind,
            )
        except Exception as exc:
            self._fail_scan_task(token, exc, operation_kind=operation_kind)
            raise

        self._record(
            "scan-pick-target",
            request_id=observation.request_id,
            selected_plan=pick_plan,
            final_status=(
                "simulated" if self.mode is RobotServiceMode.DRY_RUN else "placed"
            ),
            stage_result=pick_result,
        )
        return ScanPositionResult(
            scan_index,
            1,
            1,
            "picked_and_placed_unverified",
        )

    def _require_scan_operation_state(
        self,
        token: _ActiveOperation,
        state: RobotServiceState,
        stage: str,
        *,
        operation_kind: str,
    ) -> None:
        if not self._set_operation_state(token, state):
            raise RobotServiceStateError(
                f"{operation_kind} was cancelled before {stage}"
            )

    def _execute_scan_motion(
        self,
        token: _ActiveOperation,
        plan: object,
        *,
        operation_kind: str,
    ) -> None:
        self._require_scan_operation_state(
            token,
            (
                RobotServiceState.EXECUTING
                if self.mode is RobotServiceMode.EXECUTE
                else RobotServiceState.PLANNING
            ),
            "motion execution",
            operation_kind=operation_kind,
        )
        self._controller.execute_base_plan(plan)

    def _fail_scan_task(
        self,
        token: _ActiveOperation,
        exc: Exception,
        *,
        operation_kind: str,
    ) -> None:
        if not self._operation_is_current(token):
            return
        self._best_effort_stop()
        self._finish_operation(
            token,
            final_state=RobotServiceState.FAULT,
            fault=f"{operation_kind}: {exc}",
        )
        self._record(operation_kind, final_status="fault", error=str(exc))

    def _rotary_holding_snapshot(self) -> bool | None:
        state_reader = getattr(self._axis_motion, "get_axis_states", None)
        if not callable(state_reader):
            return None
        try:
            states = state_reader(_ALL_AXIS_NAMES)
        except Exception:
            return None
        if not isinstance(states, tuple) or not all(
            isinstance(state, AxisState) for state in states
        ):
            return None
        by_axis = {state.axis: state for state in states}
        rotary = (
            AxisName.SHOULDER,
            AxisName.ELBOW,
            AxisName.ROTATION,
        )
        if not all(axis in by_axis for axis in rotary):
            return None
        enabled = tuple(by_axis[axis].enabled for axis in rotary)
        if enabled == (True, True, True):
            return True
        if enabled == (False, False, False):
            return False
        return None

    def _require_enabled_stationary_axes(self) -> None:
        state_reader = getattr(self._axis_motion, "get_axis_states", None)
        if not callable(state_reader):
            raise RobotServiceCapabilityError(
                "axis state query is required to validate enabled joints"
            )
        states = state_reader(_ALL_AXIS_NAMES)
        if (
            not isinstance(states, tuple)
            or len(states) != len(_ALL_AXIS_NAMES)
            or not all(isinstance(state, AxisState) for state in states)
        ):
            raise RobotServiceStateError(
                "enabled joint validation did not return all axis states"
            )
        by_axis = {state.axis: state for state in states}
        if set(by_axis) != set(_ALL_AXIS_NAMES):
            raise RobotServiceStateError(
                "enabled joint validation did not return all axis states"
            )
        if any(
            not state.connected
            or state.busy is not False
            or state.faulted
            or not state.position_valid
            for state in states
        ):
            raise RobotServiceStateError(
                "enabled joint validation requires connected, stationary, "
                "fault-free axes with valid positions"
            )
        if any(
            by_axis[axis].enabled is not True
            for axis in (
                AxisName.SHOULDER,
                AxisName.ELBOW,
                AxisName.ROTATION,
            )
        ):
            raise RobotServiceStateError(
                "rotary joint holding was not confirmed after enable"
            )

    def _read_current_axis_state(self) -> RobotAxisState:
        state_reader = getattr(self._axis_motion, "get_axis_states", None)
        if not callable(state_reader):
            raise RobotServiceCapabilityError(
                "Current real axis state is unavailable in this Robot Service mode."
            )
        states = state_reader(_ALL_AXIS_NAMES)
        if not isinstance(states, tuple) or not all(
            isinstance(state, AxisState) for state in states
        ):
            raise RobotServiceStateError(
                "Current five-axis positions are unavailable or invalid."
            )
        state_by_axis = {state.axis: state for state in states}
        if len(states) != len(_ALL_AXIS_NAMES) or set(state_by_axis) != set(
            _ALL_AXIS_NAMES
        ):
            raise RobotServiceStateError(
                "Current five-axis positions are unavailable or invalid."
            )
        if any(state_by_axis[axis].busy is not False for axis in _ALL_AXIS_NAMES):
            raise RobotServiceStateError(
                "Robot must be stationary before resolving a camera point."
            )

        positions: dict[AxisName, float] = {}
        for axis in _ALL_AXIS_NAMES:
            state = state_by_axis[axis]
            position = state.current_position
            if (
                not state.connected
                or state.faulted
                or not state.position_valid
                or position is None
                or isinstance(position, bool)
                or not isinstance(position, (int, float))
                or not math.isfinite(position)
                or state.position_unit != _EXPECTED_POSITION_UNITS[axis]
            ):
                raise RobotServiceStateError(
                    "Current five-axis positions are unavailable or invalid."
                )
            positions[axis] = float(position)
        return RobotAxisState(
            slide_mm=positions[AxisName.SLIDE],
            z_mm=positions[AxisName.Z],
            shoulder_deg=positions[AxisName.SHOULDER],
            elbow_deg=positions[AxisName.ELBOW],
            rotation_deg=positions[AxisName.ROTATION],
        )

    def _best_effort_stop(self) -> None:
        try:
            self._controller.stop()
        except Exception:
            pass

    def _fault(self, operation: str, exc: Exception) -> None:
        with self._state_lock:
            self.fault = f"{operation}: {exc}"
            self.state = RobotServiceState.FAULT
        self._record(operation, final_status="fault", error=str(exc))

    def _record(self, operation: str, **fields: object) -> None:
        with self._state_lock:
            state = self.state
        self.recorder.record(operation, application_state=state, **fields)


def _finite_coordinate(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite real number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _finite_positive_optional(name: str, value: object | None) -> None:
    if value is None:
        return
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite positive number")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


_PRE_SUBMISSION_CODES = {
    MotionErrorCode.INVALID_REQUEST,
    MotionErrorCode.UNKNOWN_AXIS,
    MotionErrorCode.UNSUPPORTED_PARAMETER,
    MotionErrorCode.UNSUPPORTED_COMMAND,
    MotionErrorCode.INVALID_STATE,
    MotionErrorCode.POSITION_INVALID,
    MotionErrorCode.NOT_HOMED,
    MotionErrorCode.SOFT_LIMIT,
    MotionErrorCode.BUSY,
}


def _is_pre_submission_rejection(exc: Exception) -> bool:
    return isinstance(exc, MotionAuthorizationError) or (
        isinstance(exc, UnifiedMotionError)
        and exc.error_code in _PRE_SUBMISSION_CODES
    )


__all__ = [
    "MotionResult", "MushroomRobotService", "ResolvedCameraPoint",
    "RobotServiceCapabilities", "RobotServiceCapabilityError", "RobotServiceError",
    "RobotServiceStateError", "RobotServiceStatus",
]
