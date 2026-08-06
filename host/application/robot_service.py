"""机器人进程级唯一入口：生命周期、状态、视觉与抓取工作流编排。"""

from __future__ import annotations

from dataclasses import dataclass
import math

from application.controller import MushroomRobotController
from application.execution_record import ExecutionRecorder, NullExecutionRecorder
from application.grasp_profile import GraspProfile
from application.motion_target import BaseToolTarget
from application.ports import _AxisMotionPort
from application.pick_planner import PickPlan
from application.pick_workflow import PickOutcome, PickResult, VisionPickWorkflow
from application.runtime_state import RobotServiceMode, RobotServiceState
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
class ResolvedCameraPoint:
    camera_point_mm: tuple[float, float, float]
    base_point_mm: tuple[float, float, float]
    frame_id: str
    tool_camera_source: str
    tool_camera_validated: bool

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
        axis_motion: _AxisMotionPort | None = None,
        recorder: ExecutionRecorder | None = None,
        activate_controller_on_startup: bool | None = None,
        vision_gateway_description: str = "unavailable",
    ) -> None:
        if not isinstance(controller, MushroomRobotController):
            raise TypeError("controller must be a MushroomRobotController")
        if workflow is not None and not isinstance(workflow, VisionPickWorkflow):
            raise TypeError("workflow must be VisionPickWorkflow or None")
        if not isinstance(mode, RobotServiceMode):
            raise TypeError("mode must be a RobotServiceMode")
        if grasp_profile is not None and not isinstance(grasp_profile, GraspProfile):
            raise TypeError("grasp_profile must be a GraspProfile or None")
        self._axis_motion = axis_motion
        self._controller = controller
        self._workflow = workflow
        self.mode = mode
        self.grasp_profile = grasp_profile
        self.recorder = recorder or NullExecutionRecorder()
        self.activate_controller_on_startup = (
            mode is not RobotServiceMode.READ_ONLY
            if activate_controller_on_startup is None
            else bool(activate_controller_on_startup)
        )
        self.vision_gateway_description = vision_gateway_description
        self.state = RobotServiceState.CREATED
        self.fault: str | None = None
        self._started_controller = False

    @property
    def capabilities(self) -> RobotServiceCapabilities:
        base = self._controller.capabilities
        vision_observation = self._workflow is not None
        pick_planning = bool(base.vision_target_resolution and self.grasp_profile is not None and vision_observation)
        axis_listing = callable(getattr(self._axis_motion, "list_axes", None))
        axis_state_query = callable(getattr(self._axis_motion, "get_axis_states", None)) and (
            self.mode is RobotServiceMode.DRY_RUN
            or (self.mode is RobotServiceMode.EXECUTE and self._started_controller)
        )
        axis_motion = (
            self.mode is RobotServiceMode.EXECUTE
            and self.state is RobotServiceState.READY
            and self._started_controller
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
        if self.state not in (RobotServiceState.CREATED, RobotServiceState.SHUTDOWN):
            raise RobotServiceStateError(f"startup requires CREATED/SHUTDOWN, got {self.state.value}")
        self.state = RobotServiceState.STARTING
        try:
            if self.activate_controller_on_startup:
                self._controller.startup()
                self._started_controller = True
            self.fault = None
            self.state = RobotServiceState.READY
            self._record("startup", final_status="ready")
        except Exception as exc:
            self._fault("startup", exc)
            raise

    def shutdown(self) -> None:
        try:
            if self._started_controller:
                self._controller.shutdown()
        finally:
            self._started_controller = False
            self.state = RobotServiceState.SHUTDOWN
            self._record("shutdown", final_status="shutdown")

    def status(self) -> RobotServiceStatus:
        backend_status = None
        if self._started_controller and self.state not in (RobotServiceState.SHUTDOWN, RobotServiceState.CREATED):
            try:
                backend_status = self._controller.get_status()
            except Exception as exc:
                backend_status = f"unavailable: {exc}"
        return RobotServiceStatus(self.state, self.mode, self.capabilities, backend_status, self.fault)

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
        try:
            self._require_execute(f"raw axis {command_kind} motion")
            _finite_positive_optional("timeout_s", timeout_s)
            if not self._started_controller or not callable(submit) or not callable(wait):
                raise RobotServiceCapabilityError(
                    f"raw axis {command_kind} motion is unavailable"
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
        assert callable(submit) and callable(wait)
        self.state = RobotServiceState.EXECUTING
        submitted = False
        try:
            handle = submit(target)
            submitted = True
            result = wait(handle, timeout_s=timeout_s)
        except Exception as exc:
            if _is_pre_submission_rejection(exc):
                self.state = RobotServiceState.READY
                final_status = "rejected"
            else:
                self._best_effort_stop_axis(target.axis)
                self.fault = f"raw axis {command_kind} motion: {exc}"
                self.state = RobotServiceState.FAULT
                final_status = "fault"
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
            self.state = RobotServiceState.READY
        else:
            self._best_effort_stop_axis(target.axis)
            self.fault = (
                f"raw axis {command_kind} motion: {result.status.value}: "
                f"{result.message}"
            )
            self.state = RobotServiceState.FAULT
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
    ) -> ResolvedCameraPoint:
        """把当前真实姿态下的 Camera 点只读转换到 Base frame。"""

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

        axis_state = self._read_current_axis_state()
        base_T_tool = resolver.pose_provider.forward_kinematics_base(axis_state)
        if not isinstance(base_T_tool, RigidTransform):
            raise RobotServiceError(
                "current pose provider returned a non-RigidTransform base_T_tool"
            )
        base_point = (
            base_T_tool @ calibration.tool_T_camera
        ).transform_point(camera_point)
        return ResolvedCameraPoint(
            camera_point_mm=camera_point,
            base_point_mm=tuple(float(value) for value in base_point),
            frame_id=frame_id,
            tool_camera_source=calibration.source,
            tool_camera_validated=calibration.validated,
        )

    def plan_base_target(self, target: BaseToolTarget) -> object:
        self._require_ready("plan")
        self._require_not_read_only("plan")
        self.state = RobotServiceState.PLANNING
        try:
            plan = self._controller.plan_base_target(target)
            self.state = RobotServiceState.READY
            self._record("plan", input_target=target, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("plan", input_target=target, final_status="rejected", error=str(exc))
            raise

    def move_base_target(self, target: BaseToolTarget) -> MotionResult:
        plan = self.plan_base_target(target)
        if self.mode is not RobotServiceMode.EXECUTE:
            return MotionResult(False, plan, "Dry-run plan complete; no motion command was submitted.")
        self.state = RobotServiceState.EXECUTING
        try:
            self._controller.execute_base_plan(plan)
            self.state = RobotServiceState.READY
            result = MotionResult(True, plan, "Base motion completed.")
            self._record("move", input_target=target, selected_plan=plan, final_status="ready")
            return result
        except Exception as exc:
            self._best_effort_stop()
            self._fault("move", exc)
            raise

    def request_observation(self) -> VisionTargetObservation:
        self._require_ready("observe")
        self._require_not_read_only("observe")
        workflow = self._require_workflow()
        self.state = RobotServiceState.OBSERVING
        try:
            observation = workflow.request_observation()
            self.state = RobotServiceState.READY
            self._record("observe", request_id=observation.request_id, final_status="ready")
            return observation
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("observe", final_status="rejected", error=str(exc))
            raise

    def plan_observation(self, observation: VisionTargetObservation, grasp_profile: GraspProfile | None = None) -> PickPlan:
        self._require_ready("plan-observation")
        self._require_not_read_only("plan-observation")
        if not self._controller.capabilities.vision_target_resolution:
            raise RobotServiceCapabilityError(
                "Hand-eye calibration is missing or not validated."
            )
        profile = self._require_profile(grasp_profile)
        workflow = self._require_workflow()
        self.state = RobotServiceState.PLANNING
        try:
            plan = workflow.plan_observation(observation, profile)
            self.state = RobotServiceState.READY
            self._record("plan-observation", request_id=observation.request_id, selected_plan=plan, final_status="ready")
            return plan
        except Exception as exc:
            self.state = RobotServiceState.READY
            self._record("plan-observation", request_id=observation.request_id, final_status="rejected", error=str(exc))
            raise

    def execute_pick_plan(self, plan: PickPlan) -> PickResult:
        self._require_ready("pick")
        workflow = self._require_workflow()
        execute = self.mode is RobotServiceMode.EXECUTE
        self.state = RobotServiceState.EXECUTING if execute else RobotServiceState.PLANNING
        result = workflow.execute_pick_plan(plan, execute=execute)
        if result.outcome is PickOutcome.FAILED:
            self.fault = result.message
            self.state = RobotServiceState.FAULT
        else:
            self.state = RobotServiceState.READY
        self._record("pick", request_id=plan.observation.request_id, selected_plan=plan, final_status=self.state.value, stage_result=result)
        return result

    def pick(self, grasp_profile: GraspProfile | None = None) -> PickResult:
        observation = self.request_observation()
        plan = self.plan_observation(observation, grasp_profile)
        return self.execute_pick_plan(plan)

    def return_to_startup(self) -> object:
        self._require_ready("return")
        self._require_execute("return")
        self.state = RobotServiceState.EXECUTING
        try:
            result = self._controller.return_to_startup()
            self.state = RobotServiceState.READY
            return result
        except Exception as exc:
            self._best_effort_stop()
            self._fault("return", exc)
            raise

    def stop(self) -> None:
        if self.mode is RobotServiceMode.EXECUTE and self._started_controller:
            try:
                self._controller.stop()
            except Exception as exc:
                self._fault("stop", exc)
                raise
        state_reader = getattr(self._axis_motion, "get_axis_states", None)
        if (
            self.mode is RobotServiceMode.EXECUTE
            and self._started_controller
            and callable(state_reader)
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
                self.fault = f"stop state verification: {exc}"
            if valid_stop:
                self.fault = None
                self.state = RobotServiceState.READY
            else:
                self.fault = self.fault or "stop did not confirm valid stationary axes"
                self.state = RobotServiceState.FAULT
        elif self.state is not RobotServiceState.FAULT:
            self.state = RobotServiceState.READY
        self._record("stop", final_status=self.state.value)

    def enable_joints(self) -> object:
        self._require_execute("joints enable")
        result = self._controller.enable_joints()
        self.state = RobotServiceState.READY
        return result

    def disable_joints(self) -> object:
        self._require_execute("joints disable")
        result = self._controller.disable_joints()
        self.state = RobotServiceState.DISABLED
        return result

    def suction(self, action: str) -> object:
        self._require_ready(f"suction {action}")
        self._require_execute(f"suction {action}")
        methods = {
            "grip": self._controller.suction_grip,
            "release": self._controller.suction_release,
            "idle": self._controller.suction_idle,
        }
        if action not in methods:
            raise ValueError("suction action must be grip, release, or idle")
        try:
            return methods[action]()
        except Exception as exc:
            self._best_effort_stop()
            self._fault(f"suction {action}", exc)
            raise

    def _require_ready(self, operation: str) -> None:
        if self.state is not RobotServiceState.READY:
            raise RobotServiceStateError(f"{operation} requires READY, got {self.state.value}")

    def _require_not_read_only(self, operation: str) -> None:
        if self.mode is RobotServiceMode.READ_ONLY:
            raise RobotServiceCapabilityError(f"{operation} is unavailable in read-only mode")

    def _require_execute(self, operation: str) -> None:
        self._require_ready(operation)
        if self.mode is not RobotServiceMode.EXECUTE:
            raise RobotServiceCapabilityError(f"{operation} requires execute mode and explicit motion authorization")

    def _require_workflow(self) -> VisionPickWorkflow:
        if self._workflow is None:
            raise RobotServiceCapabilityError("Vision gateway/workflow is unavailable.")
        return self._workflow

    def _require_profile(self, provided: GraspProfile | None) -> GraspProfile:
        profile = provided or self.grasp_profile
        if profile is None:
            raise RobotServiceCapabilityError("Grasp profile is missing or not validated.")
        return profile

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
        self.fault = f"{operation}: {exc}"
        self.state = RobotServiceState.FAULT
        self._record(operation, final_status="fault", error=str(exc))

    def _record(self, operation: str, **fields: object) -> None:
        self.recorder.record(operation, application_state=self.state, **fields)


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
