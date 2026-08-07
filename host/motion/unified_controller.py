"""Thin unified asynchronous point-to-point controller for five robot axes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from collections.abc import Callable, Mapping
from uuid import uuid4

from config.project.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.project.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from drivers.can_bus import CanBusNotOpenError, MotorCommunicationError
from drivers.feetech_protocol import (
    FeetechDeviceError,
    FeetechError,
    FeetechNotOpenError,
    FeetechProtocolError,
    FeetechTimeoutError,
)
from drivers.stm32_motion import (
    STM32AxisFault,
    STM32CommandError,
    STM32CommandSubmission,
    STM32MotionConfigurationError,
    STM32MotionError,
    STM32MotionProtocolError,
    STM32MotionTimeoutError,
)
from motion.authorization import MotionAuthorization
from motion.suction import SuctionStatus
from motion.unified_protocol import (
    ArrivalConfig,
    AxisCapabilities,
    AxisDescriptor,
    AxisKind,
    AxisName,
    AxisState,
    AxisTarget,
    MotionCommandHandle,
    MotionCommandResult,
    MotionCommandStatus,
    MotionErrorCode,
    MultiAxisCommandHandle,
    MultiAxisCommandResult,
    MultiAxisTarget,
    RelativeAxisTarget,
    RotaryJointEnableStatus,
    StopReport,
)
from robot.feetech_rotation import (
    FeetechRotationError,
    FeetechRotationLimitError,
    FeetechRotationPositionError,
    position_rad_to_raw,
)
from robot.joint import (
    JointError,
    JointInitializationError,
    JointLimitError,
    JointMotorDisabledError,
    JointMotorFaultError,
    JointMotorMovingError,
    JointPositionOutOfRangeError,
)


_AXIS_ORDER = tuple(AxisName)
_LINEAR_AXES = frozenset((AxisName.SLIDE, AxisName.Z))
_CAN_AXES = frozenset((AxisName.SHOULDER, AxisName.ELBOW))
_ROTARY_AXES = (AxisName.SHOULDER, AxisName.ELBOW, AxisName.ROTATION)
_POSITION_HOLD_CONFIRM_TIMEOUT_S = 2.0
_TERMINAL_STATUSES = frozenset(
    (
        MotionCommandStatus.ARRIVED,
        MotionCommandStatus.REJECTED,
        MotionCommandStatus.ABORTED,
        MotionCommandStatus.TIMEOUT,
        MotionCommandStatus.FAULT,
        MotionCommandStatus.COMMUNICATION_ERROR,
    )
)
_INT32_MIN = -(2**31)
_INT32_MAX = 2**31 - 1
_UINT32_MAX = 2**32 - 1

_STM32_AXIS_FAULT_NAMES = {
    STM32AxisFault.LIMIT: "stm32_axis.limit",
    STM32AxisFault.POSITION_INVALID: "stm32_axis.position_invalid",
    STM32AxisFault.HARDWARE_OR_CONFIG: "stm32_axis.hardware_or_config",
    STM32AxisFault.HOMING: "stm32_axis.homing",
}


class UnifiedMotionError(RuntimeError):
    """Stable public exception raised before a command handle can be returned."""

    def __init__(
        self,
        error_code: MotionErrorCode,
        message: str,
        *,
        axis: AxisName | None = None,
    ) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.axis = axis


class MultiAxisSubmissionError(UnifiedMotionError):
    """A group submission failed after zero or more axes were accepted."""

    def __init__(
        self,
        error_code: MotionErrorCode,
        message: str,
        *,
        axis: AxisName,
        result: MultiAxisCommandResult,
    ) -> None:
        super().__init__(error_code, message, axis=axis)
        self.result = result


@dataclass
class _CommandRecord:
    handle: MotionCommandHandle
    status: MotionCommandStatus
    accepted_at: float
    target_position: float
    stable_since: float | None
    terminal_result: MotionCommandResult | None
    backend_token: object | None
    is_home: bool = False
    stop_result: MotionCommandResult | None = None


class UnifiedMotionController:
    """Dispatch logical mm/deg targets without opening or enabling hardware."""

    def __init__(
        self,
        *,
        stm32_client: object | None,
        shoulder_joint: object | None,
        elbow_joint: object | None,
        rotation_axis: object | None,
        linear_position_limits: Mapping[AxisName, tuple[float, float]],
        linear_motion_limits: Mapping[AxisName, tuple[float, float]],
        arrival_configs: Mapping[AxisName, ArrivalConfig],
        axis_descriptors: Mapping[AxisName, AxisDescriptor] | None = None,
        default_motion_parameters: Mapping[
            AxisName, tuple[float | None, float | None]
        ]
        | None = None,
        authorization: MotionAuthorization | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        max_command_history: int = 1024,
        suction: object | None = None,
    ) -> None:
        if max_command_history < 1:
            raise ValueError("max_command_history must be positive")
        self._backends = {
            AxisName.SLIDE: stm32_client,
            AxisName.Z: stm32_client,
            AxisName.SHOULDER: shoulder_joint,
            AxisName.ELBOW: elbow_joint,
            AxisName.ROTATION: rotation_axis,
        }
        self._arrival_configs = self._validate_arrival_configs(arrival_configs)
        self._linear_position_limits = self._validate_linear_position_limits(
            linear_position_limits
        )
        self._linear_motion_limits = self._validate_linear_motion_limits(
            linear_motion_limits
        )
        self._descriptors = self._build_descriptors(axis_descriptors)
        self._default_motion_parameters = dict(default_motion_parameters or {})
        self.authorization = authorization or MotionAuthorization()
        if not isinstance(self.authorization, MotionAuthorization):
            raise TypeError("authorization must be MotionAuthorization")
        self._clock = clock
        self._sleep = sleep
        self._max_command_history = max_command_history
        self._suction = suction
        self._records: dict[str, _CommandRecord] = {}
        self._active_by_axis: dict[AxisName, str] = {}
        self._lock = threading.RLock()

    def list_axes(self) -> tuple[AxisDescriptor, ...]:
        return tuple(self._descriptors[axis] for axis in _AXIS_ORDER)

    def describe_axis(self, axis: AxisName) -> AxisDescriptor:
        axis = self._require_axis(axis)
        return self._descriptors[axis]

    def get_state(self, axis: AxisName) -> AxisState:
        axis = self._require_axis(axis)
        backend = self._backends[axis]
        descriptor = self._descriptors[axis]
        if backend is None:
            return AxisState(
                axis=axis,
                connected=False,
                enabled=None,
                busy=None,
                homed=None,
                position_valid=False,
                current_position=None,
                position_unit=descriptor.position_unit,
                faulted=False,
                fault_code=None,
                fault_message="backend is not configured",
            )
        try:
            if axis in _LINEAR_AXES:
                state = backend.query_axis(axis.value)
                return AxisState(
                    axis=axis,
                    connected=True,
                    enabled=state.enabled,
                    busy=state.busy,
                    homed=state.homed,
                    position_valid=state.position_valid,
                    current_position=(
                        _micrometres_to_millimetres(state.position_um)
                        if state.position_valid
                        else None
                    ),
                    position_unit="mm",
                    faulted=state.fault != 0,
                    fault_code=state.fault or None,
                    fault_message=(
                        f"{_stm32_axis_fault_name(state.fault)} "
                        f"(fault_code={state.fault})"
                        if state.fault
                        else None
                    ),
                )
            if axis in _CAN_AXES:
                state = backend.get_state()
                fault_code = getattr(state, "error_state", None)
                motor_state = getattr(state, "motor_state", None)
                moving = getattr(state, "moving", None)
                return AxisState(
                    axis=axis,
                    connected=True,
                    enabled=(motor_state == 0 if motor_state is not None else None),
                    busy=moving if isinstance(moving, bool) else None,
                    homed=None,
                    position_valid=bool(state.position_valid),
                    current_position=(
                        math.degrees(state.position_rad)
                        if state.position_valid
                        else None
                    ),
                    position_unit="deg",
                    faulted=fault_code not in (None, 0),
                    fault_code=fault_code or None,
                    fault_message=(
                        f"MG4010 device fault {fault_code}"
                        if fault_code not in (None, 0)
                        else None
                    ),
                )
            enabled = backend.torque_enabled()
            feedback = backend.read_feedback()
            return AxisState(
                axis=axis,
                connected=True,
                enabled=enabled,
                busy=(
                    feedback.moving
                    if isinstance(getattr(feedback, "moving", None), bool)
                    else None
                ),
                homed=None,
                position_valid=True,
                current_position=math.degrees(feedback.position_rad),
                position_unit="deg",
                faulted=feedback.error_raw != 0,
                fault_code=feedback.error_raw or None,
                fault_message=(
                    f"Feetech feedback error {feedback.error_raw}"
                    if feedback.error_raw
                    else None
                ),
            )
        except Exception as exc:
            code = self._map_exception(exc)
            raise UnifiedMotionError(code, str(exc), axis=axis) from exc

    def get_axis_states(
        self,
        axes: tuple[AxisName, ...] | None = None,
    ) -> tuple[AxisState, ...]:
        """Return logical states in caller order, or all axes in canonical order."""

        selected = _AXIS_ORDER if axes is None else axes
        if not isinstance(selected, tuple):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "axes must be a tuple of AxisName values or None",
            )
        return tuple(self.get_state(axis) for axis in selected)

    def suction_grip(self) -> SuctionStatus:
        """执行吸附并返回 STM32 已确认的输出命令状态。"""

        self.authorization.require_motion()
        suction = self._require_suction()
        try:
            return suction.grip()
        except Exception as exc:
            raise UnifiedMotionError(self._map_exception(exc), str(exc)) from exc

    def suction_release(self) -> SuctionStatus:
        """执行固件定义的定时释放并返回输出命令状态。"""

        self.authorization.require_motion()
        suction = self._require_suction()
        try:
            return suction.release()
        except Exception as exc:
            raise UnifiedMotionError(self._map_exception(exc), str(exc)) from exc

    def suction_idle(self) -> SuctionStatus:
        """关闭泵和释放阀并返回输出命令状态。"""

        self.authorization.require_motion()
        suction = self._require_suction()
        try:
            return suction.idle()
        except Exception as exc:
            raise UnifiedMotionError(self._map_exception(exc), str(exc)) from exc

    def get_suction_status(self) -> SuctionStatus:
        """读取 commanded output state；不声称检测到物理真空。"""

        suction = self._require_suction()
        try:
            return suction.get_status()
        except Exception as exc:
            raise UnifiedMotionError(self._map_exception(exc), str(exc)) from exc

    def get_rotary_joint_enable_status(self) -> RotaryJointEnableStatus:
        """读取 Shoulder、Elbow 和 Rotation 的真实使能状态。"""

        shoulder = self._backends[AxisName.SHOULDER]
        elbow = self._backends[AxisName.ELBOW]
        rotation = self._backends[AxisName.ROTATION]
        try:
            if shoulder is None or elbow is None or rotation is None:
                return RotaryJointEnableStatus(
                    None if shoulder is None else self._joint_enabled(shoulder),
                    None if elbow is None else self._joint_enabled(elbow),
                    None if rotation is None else self._rotation_enabled(rotation),
                )
            return RotaryJointEnableStatus(
                self._joint_enabled(shoulder),
                self._joint_enabled(elbow),
                self._rotation_enabled(rotation),
            )
        except Exception as exc:
            raise UnifiedMotionError(self._map_exception(exc), str(exc)) from exc

    def rotary_joints_enabled(self) -> bool:
        """仅当三个旋转关节都明确报告使能时返回 True。"""

        return self.get_rotary_joint_enable_status().all_enabled

    def enable_rotary_joints(self) -> RotaryJointEnableStatus:
        """按 Shoulder→Elbow→Rotation 使能并重新建立真实位置。"""

        self.authorization.require_motion()
        backends = self._require_rotary_backends()
        newly_enabled: list[AxisName] = []
        try:
            for axis in _ROTARY_AXES:
                backend = backends[axis]
                enabled = (
                    self._rotation_enabled(backend)
                    if axis is AxisName.ROTATION
                    else self._joint_enabled(backend)
                )
                if enabled:
                    continue
                if axis is AxisName.ROTATION:
                    # Torque OFF 时先写入当前角目标，避免使能后追逐旧目标。
                    feedback = backend.read_feedback()
                    backend.command_position(
                        feedback.position_rad,
                        backend.config.max_speed_raw,
                    )
                    newly_enabled.append(axis)
                    backend.enable_torque()
                    if not self._rotation_enabled(backend):
                        raise FeetechRotationError(
                            "Rotation torque enable was acknowledged but is not active"
                        )
                else:
                    newly_enabled.append(axis)
                    backend.enable()
                    if not self._joint_enabled(backend):
                        raise JointMotorDisabledError(
                            f"{axis.value} enable was acknowledged but is not active"
                        )
            # 0x80 会清除 MG4010 运行圈数/旧命令；每次显式 enable 后都重新
            # 读取有限行程绝对位置。Rotation 同样读取实时反馈，不使用缓存。
            backends[AxisName.SHOULDER].initialize()
            backends[AxisName.ELBOW].initialize()
            backends[AxisName.ROTATION].read_feedback()
            status = self.get_rotary_joint_enable_status()
            if not status.all_enabled:
                raise UnifiedMotionError(
                    MotionErrorCode.BACKEND_ERROR,
                    "one or more rotary joints did not confirm enabled state",
                )
            return status
        except Exception as exc:
            for axis in reversed(newly_enabled):
                backend = backends[axis]
                try:
                    if axis is AxisName.ROTATION:
                        backend.disable_torque()
                    else:
                        backend.disable()
                except Exception:
                    pass
            if isinstance(exc, UnifiedMotionError):
                raise
            raise UnifiedMotionError(
                self._map_exception(exc),
                f"failed to enable rotary joints; best-effort rollback attempted: {exc}",
            ) from exc

    def disable_rotary_joints(
        self,
        *,
        stop_timeout_s: float = 2.0,
    ) -> RotaryJointEnableStatus:
        """确认静止后按 Rotation→Elbow→Shoulder 移除保持力。"""

        self.authorization.require_motion()
        if not math.isfinite(stop_timeout_s) or stop_timeout_s <= 0:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "stop_timeout_s must be finite and positive",
            )
        backends = self._require_rotary_backends()
        try:
            current = self.get_rotary_joint_enable_status()
            if current == RotaryJointEnableStatus(False, False, False):
                return current

            rotation_feedback = backends[AxisName.ROTATION].read_feedback()
            if rotation_feedback.moving:
                raise UnifiedMotionError(
                    MotionErrorCode.BUSY,
                    "Rotation is moving and has no verified independent stop; "
                    "refusing to disable joint torque",
                    axis=AxisName.ROTATION,
                )

            moving_linear: list[AxisName] = []
            for axis in (AxisName.Z, AxisName.SLIDE):
                state = self.get_state(axis)
                if state.busy is None:
                    raise UnifiedMotionError(
                        MotionErrorCode.BUSY,
                        f"cannot confirm {axis.value} is stationary before "
                        "disabling rotary-joint torque",
                        axis=axis,
                    )
                if state.busy:
                    self._require_backend(axis).stop(axis.value)
                    moving_linear.append(axis)

            moving_can: list[AxisName] = []
            for axis in _CAN_AXES:
                backend = backends[axis]
                if self._joint_enabled(backend) and backend.is_moving():
                    backend.stop()
                    moving_can.append(axis)

            deadline = self._clock() + stop_timeout_s
            while moving_linear or moving_can:
                moving_linear = [
                    axis
                    for axis in moving_linear
                    if self.get_state(axis).busy is not False
                ]
                moving_can = [
                    axis
                    for axis in moving_can
                    if backends[axis].is_moving()
                ]
                if not moving_linear and not moving_can:
                    break
                if self._clock() >= deadline:
                    names = ", ".join(
                        axis.value for axis in (*moving_linear, *moving_can)
                    )
                    raise UnifiedMotionError(
                        MotionErrorCode.BUSY,
                        f"cannot confirm stationary rotary joints: {names}",
                    )
                self._sleep(0.02)

            for axis in reversed(_ROTARY_AXES):
                backend = backends[axis]
                enabled = (
                    self._rotation_enabled(backend)
                    if axis is AxisName.ROTATION
                    else self._joint_enabled(backend)
                )
                if not enabled:
                    continue
                if axis is AxisName.ROTATION:
                    backend.disable_torque()
                    if self._rotation_enabled(backend):
                        raise FeetechRotationError(
                            "Rotation torque disable was acknowledged but remains active"
                        )
                else:
                    backend.disable()
                    if self._joint_enabled(backend):
                        raise JointMotorDisabledError(
                            f"{axis.value} disable was acknowledged but remains active"
                        )
            return self.get_rotary_joint_enable_status()
        except UnifiedMotionError:
            raise
        except Exception as exc:
            raise UnifiedMotionError(
                self._map_exception(exc),
                f"failed to disable rotary joints safely: {exc}",
            ) from exc

    def submit_absolute(self, target: AxisTarget) -> MotionCommandHandle:
        if isinstance(target, AxisTarget):
            self.authorization.require_axis_motion(target.axis)
        else:
            self.authorization.require_motion()
        with self._lock:
            target = self._validate_target(target)
            if target.axis in _ROTARY_AXES:
                self._require_rotary_motion_enabled()
            return self._submit_absolute_locked(target)

    def submit_relative(
        self,
        target: RelativeAxisTarget,
    ) -> MotionCommandHandle:
        """把当前有效逻辑位置加增量后，通过绝对位置通路原子提交。"""

        if isinstance(target, RelativeAxisTarget):
            self.authorization.require_axis_motion(target.axis)
        else:
            self.authorization.require_motion()
        if not isinstance(target, RelativeAxisTarget):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "target must be a RelativeAxisTarget",
            )
        with self._lock:
            axis = self._require_axis(target.axis)
            self._ensure_axis_idle(axis)
            if axis in _ROTARY_AXES:
                self._require_rotary_motion_enabled()
            start_position = self._read_valid_relative_start_locked(axis)
            absolute = self._validate_target(
                AxisTarget(
                    axis=axis,
                    position=start_position + target.delta,
                    velocity=target.velocity,
                    acceleration=target.acceleration,
                )
            )
            if abs(target.delta) <= self._arrival_configs[axis].position_tolerance:
                return self._record_relative_no_op_locked(absolute, start_position)
            return self._submit_absolute_locked(absolute, idle_checked=True)

    def submit_positions(self, target: MultiAxisTarget) -> MultiAxisCommandHandle:
        self.authorization.require_motion()
        validated = self._validate_positions(target)
        for item in target.targets:
            self.authorization.require_axis_motion(item.axis)
        if any(item.axis in _ROTARY_AXES for item in validated):
            self._require_rotary_motion_enabled()
        group_id = uuid4().hex
        with self._lock:
            for item in validated:
                self._ensure_axis_idle(item.axis)
            submitted: list[MotionCommandHandle] = []
            try:
                validated = self._coordinate_default_can_velocities(validated)
                for item in validated:
                    submitted.append(self._submit_validated(item))
            except UnifiedMotionError as exc:
                result = self._group_submission_failure(
                    group_id,
                    validated,
                    submitted,
                    exc,
                )
                raise MultiAxisSubmissionError(
                    exc.error_code,
                    str(exc),
                    axis=exc.axis or validated[len(submitted)].axis,
                    result=result,
                ) from exc
        return MultiAxisCommandHandle(group_id, tuple(submitted))

    def validate_positions(self, target: MultiAxisTarget) -> None:
        """Validate a complete group without sending control I/O."""

        self._validate_positions(target)

    def get_command_result(
        self,
        handle: MotionCommandHandle,
    ) -> MotionCommandResult:
        with self._lock:
            record = self._get_record(handle)
            if record.terminal_result is not None:
                return record.terminal_result
            try:
                if handle.axis in _LINEAR_AXES:
                    return self._refresh_stm32(record)
                return self._refresh_position_axis(record)
            except UnifiedMotionError as exc:
                status = self._status_for_error(exc.error_code)
                result = self._result(
                    record,
                    status,
                    error_code=exc.error_code,
                    message=str(exc),
                )
                return self._finish(record, result)
            except Exception as exc:
                code = self._map_exception(exc)
                status = self._status_for_error(code)
                result = self._result(
                    record,
                    status,
                    error_code=code,
                    message=str(exc),
                )
                if status == MotionCommandStatus.FAULT:
                    record.stop_result = self.stop(handle.axis)
                return self._finish(record, result)

    def get_group_result(
        self,
        handle: MultiAxisCommandHandle,
    ) -> MultiAxisCommandResult:
        """Poll and aggregate a submitted group without waiting for arrival."""

        if not isinstance(handle, MultiAxisCommandHandle) or not handle.commands:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "handle must be a non-empty MultiAxisCommandHandle",
            )
        results = tuple(self.get_command_result(item) for item in handle.commands)
        failure = next(
            (
                item
                for item in results
                if item.status
                in (
                    MotionCommandStatus.REJECTED,
                    MotionCommandStatus.ABORTED,
                    MotionCommandStatus.TIMEOUT,
                    MotionCommandStatus.FAULT,
                    MotionCommandStatus.COMMUNICATION_ERROR,
                )
            ),
            None,
        )
        if failure is not None:
            return MultiAxisCommandResult(
                group_id=handle.group_id,
                status=failure.status,
                results=results,
                accepted=failure.status != MotionCommandStatus.REJECTED,
                completed=False,
                message=f"axis {failure.axis.value} reported {failure.status.value}",
            )
        if all(item.status == MotionCommandStatus.ARRIVED for item in results):
            return MultiAxisCommandResult(
                group_id=handle.group_id,
                status=MotionCommandStatus.ARRIVED,
                results=results,
                accepted=True,
                completed=True,
                message="all participating axes arrived",
            )
        status = (
            MotionCommandStatus.MOVING
            if any(item.status == MotionCommandStatus.MOVING for item in results)
            else MotionCommandStatus.ACCEPTED
        )
        return MultiAxisCommandResult(
            group_id=handle.group_id,
            status=status,
            results=results,
            accepted=True,
            completed=None,
            message="group remains in progress",
        )

    def wait(
        self,
        handle: MotionCommandHandle,
        *,
        timeout_s: float | None = None,
    ) -> MotionCommandResult:
        config = self._arrival_configs[handle.axis]
        timeout = self._validated_timeout(timeout_s, config.default_timeout_s)
        deadline = self._clock() + timeout
        while True:
            result = self.get_command_result(handle)
            if result.status in _TERMINAL_STATUSES:
                if result.status in (
                    MotionCommandStatus.FAULT,
                    MotionCommandStatus.COMMUNICATION_ERROR,
                ):
                    record = self._get_record(handle)
                    if record.stop_result is None:
                        record.stop_result = self.stop(handle.axis)
                return result
            if self._clock() >= deadline:
                return self._timeout_record(handle)
            self._sleep(config.poll_interval_s)

    def wait_group(
        self,
        handle: MultiAxisCommandHandle,
        *,
        timeout_s: float | None = None,
    ) -> MultiAxisCommandResult:
        if not handle.commands:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "group handle contains no commands",
            )
        default_timeout = max(
            self._arrival_configs[item.axis].default_timeout_s
            for item in handle.commands
        )
        timeout = self._validated_timeout(timeout_s, default_timeout)
        deadline = self._clock() + timeout
        poll_interval = min(
            self._arrival_configs[item.axis].poll_interval_s
            for item in handle.commands
        )
        while True:
            results = tuple(self.get_command_result(item) for item in handle.commands)
            failure = next(
                (
                    item
                    for item in results
                    if item.status
                    in (
                        MotionCommandStatus.ABORTED,
                        MotionCommandStatus.FAULT,
                        MotionCommandStatus.COMMUNICATION_ERROR,
                    )
                ),
                None,
            )
            if failure is not None:
                prior_results = tuple(
                    record.stop_result
                    for item in handle.commands
                    if (record := self._get_record(item)).stop_result is not None
                )
                stop_report = self.stop_axes(
                    [item.axis for item in handle.commands],
                    prior=StopReport(prior_results),
                )
                self._abort_group_peers(handle, failure.axis, stop_report)
                final_results = tuple(
                    self.get_command_result(item) for item in handle.commands
                )
                return MultiAxisCommandResult(
                    group_id=handle.group_id,
                    status=failure.status,
                    results=final_results,
                    accepted=True,
                    completed=False,
                    message=(
                        f"axis {failure.axis.value} failed; one coordinated stop "
                        "attempt was made for each participating axis"
                    ),
                    stop_report=stop_report,
                )
            if all(item.status == MotionCommandStatus.ARRIVED for item in results):
                return MultiAxisCommandResult(
                    group_id=handle.group_id,
                    status=MotionCommandStatus.ARRIVED,
                    results=results,
                    accepted=True,
                    completed=True,
                    message="all participating axes arrived",
                )
            if self._clock() >= deadline:
                unfinished = [
                    command
                    for command in handle.commands
                    if self.get_command_result(command).status not in _TERMINAL_STATUSES
                ]
                stop_report = self.stop_axes([item.axis for item in unfinished])
                stop_by_axis = {item.axis: item for item in stop_report.results}
                for command in unfinished:
                    self._timeout_record(
                        command, stop_result=stop_by_axis.get(command.axis)
                    )
                results = tuple(
                    self.get_command_result(item) for item in handle.commands
                )
                return MultiAxisCommandResult(
                    group_id=handle.group_id,
                    status=MotionCommandStatus.TIMEOUT,
                    results=results,
                    accepted=True,
                    completed=False,
                    message=(
                        "group deadline expired; one coordinated stop attempt was "
                        "made for each unfinished axis"
                    ),
                    stop_report=stop_report,
                )
            self._sleep(poll_interval)

    def stop(self, axis: AxisName) -> MotionCommandResult:
        axis = self._require_axis(axis)
        command_id = uuid4().hex
        stop_method = (
            "stm32_software_stop"
            if axis in _LINEAR_AXES
            else "current_position_hold"
        )
        if not self._descriptors[axis].capabilities.stop:
            return MotionCommandResult(
                command_id=command_id,
                axis=axis,
                status=MotionCommandStatus.REJECTED,
                accepted=False,
                completed=False,
                target_position=0.0,
                final_position=None,
                position_error=None,
                error_code=MotionErrorCode.UNSUPPORTED_COMMAND,
                message=f"axis {axis.value} has no independent stop command",
                stop_method=stop_method,
                command_submitted=False,
            )
        backend = self._backends[axis]
        if backend is None:
            return MotionCommandResult(
                command_id=command_id,
                axis=axis,
                status=MotionCommandStatus.REJECTED,
                accepted=False,
                completed=False,
                target_position=0.0,
                final_position=None,
                position_error=None,
                error_code=MotionErrorCode.BACKEND_UNAVAILABLE,
                message=f"axis {axis.value} backend is unavailable",
                stop_method=stop_method,
                command_submitted=False,
            )
        target_position = 0.0
        final_position = None
        stop_submitted: bool | None = False
        try:
            if axis in _LINEAR_AXES:
                backend.stop(axis.value)
                stop_submitted = True
            elif axis is AxisName.ROTATION:
                held_position = backend.stop()
                stop_submitted = True
                target_position = math.degrees(held_position)
                deadline = self._clock() + _POSITION_HOLD_CONFIRM_TIMEOUT_S
                poll_interval = self._arrival_configs[axis].poll_interval_s
                while True:
                    feedback = backend.read_feedback()
                    if feedback.error_raw:
                        raise FeetechRotationError(
                            f"Rotation device error {feedback.error_raw} while "
                            "confirming current-position hold"
                        )
                    if not feedback.moving:
                        final_position = math.degrees(feedback.position_rad)
                        break
                    if self._clock() >= deadline:
                        return MotionCommandResult(
                            command_id=command_id,
                            axis=axis,
                            status=MotionCommandStatus.TIMEOUT,
                            accepted=True,
                            completed=False,
                            target_position=target_position,
                            final_position=math.degrees(feedback.position_rad),
                            position_error=None,
                            error_code=MotionErrorCode.TIMEOUT,
                            message=(
                                "Rotation current-position hold was submitted, "
                                "but stationary feedback was not confirmed within 2 seconds"
                            ),
                            stop_method=stop_method,
                            command_submitted=True,
                        )
                    self._sleep(poll_interval)
            else:
                hold = backend.stop()
                stop_submitted = True
                hold_position_rad = getattr(hold, "target_position_rad", None)
                if hold_position_rad is None:
                    # 兼容只实现旧 stop() 测试接口的外部后端；正式 CAN
                    # 后端始终返回 JointHoldSnapshot。
                    hold_position_rad = backend.get_state().position_rad
                target_position = math.degrees(hold_position_rad)
                deadline = self._clock() + _POSITION_HOLD_CONFIRM_TIMEOUT_S
                config = self._arrival_configs[axis]
                stable_since: float | None = None
                while True:
                    state = backend.get_state()
                    if state.error_state != 0:
                        raise JointMotorFaultError(
                            f"axis {axis.value} faulted while confirming position hold: "
                            f"0x{state.error_state:02X}"
                        )
                    if state.motor_state != 0x00:
                        raise JointMotorDisabledError(
                            f"axis {axis.value} protocol enabled state was lost "
                            "while confirming position hold"
                        )
                    current_deg = math.degrees(state.position_rad)
                    position_error = abs(current_deg - target_position)
                    if not state.moving and position_error <= config.position_tolerance:
                        if stable_since is None:
                            stable_since = self._clock()
                        if self._clock() - stable_since >= config.stable_time_s:
                            final_position = current_deg
                            break
                    else:
                        stable_since = None
                    if self._clock() >= deadline:
                        return MotionCommandResult(
                            command_id=command_id,
                            axis=axis,
                            status=MotionCommandStatus.TIMEOUT,
                            accepted=True,
                            completed=False,
                            target_position=target_position,
                            final_position=current_deg,
                            position_error=position_error,
                            error_code=MotionErrorCode.TIMEOUT,
                            message=(
                                f"{axis.value} current-position hold was submitted, "
                                "but stable stationary feedback was not confirmed "
                                "within 2 seconds; no 0x81 fallback was sent"
                            ),
                            stop_method=stop_method,
                            command_submitted=True,
                        )
                    self._sleep(config.poll_interval_s)
        except Exception as exc:
            code = self._map_exception(exc)
            status = self._status_for_error(code)
            reported_submitted = stop_submitted
            if stop_submitted is False and not isinstance(
                exc,
                (
                    JointInitializationError,
                    JointMotorDisabledError,
                    JointMotorFaultError,
                    JointPositionOutOfRangeError,
                ),
            ):
                reported_submitted = None
            return MotionCommandResult(
                command_id=command_id,
                axis=axis,
                status=status,
                accepted=True,
                completed=False,
                target_position=target_position,
                final_position=final_position,
                position_error=None,
                error_code=code,
                message=f"current-position hold failed: {exc}; no 0x81 fallback was sent",
                stop_method=stop_method,
                command_submitted=reported_submitted,
            )
        return MotionCommandResult(
            command_id=command_id,
            axis=axis,
            status=MotionCommandStatus.ABORTED,
            accepted=True,
            completed=False,
            target_position=target_position,
            final_position=final_position,
            position_error=None,
            error_code=MotionErrorCode.BACKEND_ERROR,
            message=(
                "Rotation current-position hold confirmed stationary; this is not "
                "an emergency stop"
                if axis is AxisName.ROTATION
                else (
                    f"{axis.value} current-position hold confirmed stationary; "
                    "protocol enabled state remains active"
                    if axis in _CAN_AXES
                    else "software stop accepted; this is not an emergency stop"
                )
            ),
            stop_method=stop_method,
            command_submitted=True,
        )

    def stop_axes(
        self,
        axes: tuple[AxisName, ...] | list[AxisName],
        *,
        prior: StopReport | None = None,
    ) -> StopReport:
        """每个不同轴最多停止一次，并合并此前已经尝试过的结果。"""

        results = list(prior.results if prior is not None else ())
        attempted = {item.axis for item in results}
        for axis in axes:
            checked = self._require_axis(axis)
            if checked in attempted:
                continue
            results.append(self.stop(checked))
            attempted.add(checked)
        return StopReport(tuple(results))

    def home_reference(
        self,
        axis: AxisName,
        *,
        timeout_s: float | None = None,
    ) -> MotionCommandResult:
        self.authorization.require_motion()
        axis = self._require_axis(axis)
        if axis not in _LINEAR_AXES:
            return MotionCommandResult(
                command_id=uuid4().hex,
                axis=axis,
                status=MotionCommandStatus.REJECTED,
                accepted=False,
                completed=False,
                target_position=0.0,
                final_position=None,
                position_error=None,
                error_code=MotionErrorCode.UNSUPPORTED_COMMAND,
                message=f"axis {axis.value} does not support reference homing",
            )
        with self._lock:
            self._ensure_axis_idle(axis)
            backend = self._require_backend(axis)
            try:
                token = backend.submit_home(axis.value)
            except Exception as exc:
                raise self._submission_error(axis, exc) from exc
            handle = MotionCommandHandle(uuid4().hex, axis, 0.0)
            self._records[handle.command_id] = _CommandRecord(
                handle=handle,
                status=MotionCommandStatus.ACCEPTED,
                accepted_at=self._clock(),
                target_position=0.0,
                stable_since=None,
                terminal_result=None,
                backend_token=token,
                is_home=True,
            )
            self._active_by_axis[axis] = handle.command_id
            self._prune_records()
        return self.wait(handle, timeout_s=timeout_s)

    def _submit_validated(self, target: AxisTarget) -> MotionCommandHandle:
        backend = self._require_backend(target.axis)
        try:
            token: object | None
            if target.axis in _LINEAR_AXES:
                velocity, acceleration = self._resolve_motion_parameters(target)
                token = backend.submit_move_absolute(
                    target.axis.value,
                    _millimetres_to_micrometres(target.position, "position"),
                    _millimetres_to_micrometres(velocity, "velocity", unsigned=True),
                    _millimetres_to_micrometres(
                        acceleration,
                        "acceleration",
                        unsigned=True,
                    ),
                )
            elif target.axis in _CAN_AXES:
                velocity, _acceleration = self._resolve_motion_parameters(target)
                token = backend.command_position(
                    math.radians(target.position),
                    math.radians(velocity),
                )
            else:
                speed_raw = backend.config.max_speed_raw
                token = backend.command_position(
                    math.radians(target.position),
                    speed_raw,
                )
        except Exception as exc:
            raise self._submission_error(target.axis, exc) from exc
        handle = MotionCommandHandle(uuid4().hex, target.axis, target.position)
        self._records[handle.command_id] = _CommandRecord(
            handle=handle,
            status=MotionCommandStatus.ACCEPTED,
            accepted_at=self._clock(),
            target_position=target.position,
            stable_since=None,
            terminal_result=None,
            backend_token=token,
        )
        self._active_by_axis[target.axis] = handle.command_id
        self._prune_records()
        return handle

    def _submit_absolute_locked(
        self,
        target: AxisTarget,
        *,
        idle_checked: bool = False,
    ) -> MotionCommandHandle:
        if not idle_checked:
            self._ensure_axis_idle(target.axis)
        return self._submit_validated(target)

    def _read_valid_relative_start_locked(self, axis: AxisName) -> float:
        state = self.get_state(axis)
        if state.faulted:
            raise UnifiedMotionError(
                MotionErrorCode.DEVICE_FAULT,
                f"axis {axis.value} has a device fault",
                axis=axis,
            )
        if state.busy is not False:
            raise UnifiedMotionError(
                MotionErrorCode.BUSY,
                f"axis {axis.value} stationary state is not confirmed false",
                axis=axis,
            )
        if axis in _LINEAR_AXES and state.homed is not True:
            raise UnifiedMotionError(
                MotionErrorCode.NOT_HOMED,
                f"axis {axis.value} is not homed",
                axis=axis,
            )
        if not state.position_valid or state.current_position is None:
            raise UnifiedMotionError(
                MotionErrorCode.POSITION_INVALID,
                f"axis {axis.value} has no valid current logical position",
                axis=axis,
            )
        if not math.isfinite(state.current_position):
            raise UnifiedMotionError(
                MotionErrorCode.POSITION_INVALID,
                f"axis {axis.value} current logical position is not finite",
                axis=axis,
            )
        return float(state.current_position)

    def _record_relative_no_op_locked(
        self,
        target: AxisTarget,
        current_position: float,
    ) -> MotionCommandHandle:
        handle = MotionCommandHandle(uuid4().hex, target.axis, target.position)
        record = _CommandRecord(
            handle=handle,
            status=MotionCommandStatus.ARRIVED,
            accepted_at=self._clock(),
            target_position=target.position,
            stable_since=self._clock(),
            terminal_result=None,
            backend_token=None,
        )
        record.terminal_result = self._result(
            record,
            MotionCommandStatus.ARRIVED,
            final_position=current_position,
            message="relative delta is within the axis tolerance; no motion submitted",
        )
        self._records[handle.command_id] = record
        self._prune_records()
        return handle

    def _validate_target(self, target: AxisTarget) -> AxisTarget:
        if not isinstance(target, AxisTarget):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "target must be an AxisTarget",
            )
        axis = self._require_axis(target.axis)
        descriptor = self._descriptors[axis]
        if not descriptor.capabilities.move_absolute:
            raise UnifiedMotionError(
                MotionErrorCode.UNSUPPORTED_COMMAND,
                f"axis {axis.value} does not support absolute motion",
                axis=axis,
            )
        if not descriptor.minimum_position <= target.position <= descriptor.maximum_position:
            raise UnifiedMotionError(
                MotionErrorCode.SOFT_LIMIT,
                f"axis {axis.value} target {target.position} is outside "
                f"[{descriptor.minimum_position}, {descriptor.maximum_position}] "
                f"{descriptor.position_unit}",
                axis=axis,
            )
        if target.velocity is not None and not descriptor.capabilities.configurable_velocity:
            raise UnifiedMotionError(
                MotionErrorCode.UNSUPPORTED_PARAMETER,
                f"axis {axis.value} does not support velocity in engineering units",
                axis=axis,
            )
        if (
            target.acceleration is not None
            and not descriptor.capabilities.configurable_acceleration
        ):
            raise UnifiedMotionError(
                MotionErrorCode.UNSUPPORTED_PARAMETER,
                f"axis {axis.value} does not support acceleration in engineering units",
                axis=axis,
            )
        self._require_backend(axis)
        backend = self._backends[axis]
        assert backend is not None
        try:
            if axis in _LINEAR_AXES:
                velocity, acceleration = self._resolve_motion_parameters(target)
                _millimetres_to_micrometres(target.position, "position")
                _millimetres_to_micrometres(velocity, "velocity", unsigned=True)
                assert acceleration is not None
                _millimetres_to_micrometres(
                    acceleration,
                    "acceleration",
                    unsigned=True,
                )
            elif axis in _CAN_AXES:
                velocity, _acceleration = self._resolve_motion_parameters(target)
                validate = getattr(backend, "validate_position_command", None)
                if validate is not None:
                    validate(math.radians(target.position), math.radians(velocity))
            else:
                position_rad_to_raw(math.radians(target.position), backend.config)
        except Exception as exc:
            raise self._submission_error(axis, exc) from exc
        return target

    def _validate_positions(
        self,
        target: MultiAxisTarget,
    ) -> tuple[AxisTarget, ...]:
        if not isinstance(target, MultiAxisTarget):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "target must be a MultiAxisTarget",
            )
        return tuple(self._validate_target(item) for item in target.targets)

    def _coordinate_default_can_velocities(
        self,
        targets: tuple[AxisTarget, ...],
    ) -> tuple[AxisTarget, ...]:
        """按肩肘角度差分配默认速度；只读反馈失败时不提交任何运动。"""

        by_axis = {target.axis: target for target in targets}
        if not _CAN_AXES.issubset(by_axis):
            return targets
        if any(by_axis[axis].velocity is not None for axis in _CAN_AXES):
            return targets

        distances: dict[AxisName, float] = {}
        velocity_caps: dict[AxisName, float] = {}
        minimum_velocities: dict[AxisName, float] = {}
        moving_axes: list[AxisName] = []

        for axis in (AxisName.SHOULDER, AxisName.ELBOW):
            target = by_axis[axis]
            state = self.get_state(axis)
            if state.faulted:
                raise UnifiedMotionError(
                    MotionErrorCode.DEVICE_FAULT,
                    f"axis {axis.value} has a device fault; coordinated motion was not submitted",
                    axis=axis,
                )
            if not state.position_valid or state.current_position is None:
                raise UnifiedMotionError(
                    MotionErrorCode.POSITION_INVALID,
                    f"axis {axis.value} has no valid position for coordinated motion",
                    axis=axis,
                )
            if state.busy is not False:
                raise UnifiedMotionError(
                    MotionErrorCode.BUSY,
                    f"axis {axis.value} stationary state is not confirmed false",
                    axis=axis,
                )
            if not math.isfinite(state.current_position):
                raise UnifiedMotionError(
                    MotionErrorCode.POSITION_INVALID,
                    f"axis {axis.value} current position is not finite",
                    axis=axis,
                )

            backend = self._require_backend(axis)
            config = getattr(backend, "config", None)
            position_tolerance_rad = getattr(config, "position_tolerance_rad", None)
            max_velocity_rad_s = getattr(config, "max_velocity_rad_s", None)
            gear_ratio = getattr(config, "gear_ratio", None)
            if not all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                and float(value) > 0.0
                for value in (
                    position_tolerance_rad,
                    max_velocity_rad_s,
                    gear_ratio,
                )
            ):
                raise UnifiedMotionError(
                    MotionErrorCode.BACKEND_ERROR,
                    f"axis {axis.value} has invalid coordination configuration",
                    axis=axis,
                )

            default_velocity, _acceleration = self._resolve_motion_parameters(target)
            maximum_velocity = math.degrees(float(max_velocity_rad_s))
            velocity_cap = min(default_velocity, maximum_velocity)
            minimum_velocity = 1.0 / float(gear_ratio)
            if minimum_velocity > velocity_cap:
                raise UnifiedMotionError(
                    MotionErrorCode.BACKEND_ERROR,
                    f"axis {axis.value} default velocity is below the A4 protocol minimum",
                    axis=axis,
                )

            distance = abs(target.position - state.current_position)
            tolerance = math.degrees(float(position_tolerance_rad))
            distances[axis] = distance
            velocity_caps[axis] = velocity_cap
            minimum_velocities[axis] = minimum_velocity
            if distance > tolerance:
                moving_axes.append(axis)

        if not moving_axes:
            return targets

        duration_s = max(
            distances[axis] / velocity_caps[axis]
            for axis in moving_axes
        )
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise UnifiedMotionError(
                MotionErrorCode.BACKEND_ERROR,
                "coordinated shoulder/elbow duration is invalid",
            )

        coordinated_velocities = {
            axis: min(
                velocity_caps[axis],
                max(minimum_velocities[axis], distances[axis] / duration_s),
            )
            for axis in moving_axes
        }
        return tuple(
            AxisTarget(
                target.axis,
                target.position,
                coordinated_velocities.get(target.axis, target.velocity),
                target.acceleration,
            )
            for target in targets
        )

    def _resolve_motion_parameters(self, target: AxisTarget) -> tuple[float, float | None]:
        default_velocity, default_acceleration = self._default_motion_parameters.get(
            target.axis,
            (None, None),
        )
        velocity = target.velocity if target.velocity is not None else default_velocity
        acceleration = (
            target.acceleration
            if target.acceleration is not None
            else default_acceleration
        )
        if velocity is None:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                f"axis {target.axis.value} requires velocity or a configured default",
                axis=target.axis,
            )
        if target.axis in _LINEAR_AXES and acceleration is None:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                f"axis {target.axis.value} requires acceleration or a configured default",
                axis=target.axis,
            )
        if not math.isfinite(velocity) or velocity <= 0:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                f"axis {target.axis.value} velocity must be finite and positive",
                axis=target.axis,
            )
        if acceleration is not None and (
            not math.isfinite(acceleration) or acceleration <= 0
        ):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                f"axis {target.axis.value} acceleration must be finite and positive",
                axis=target.axis,
            )
        if target.axis in _LINEAR_AXES:
            maximum_velocity, maximum_acceleration = self._linear_motion_limits[
                target.axis
            ]
            if velocity > maximum_velocity:
                raise UnifiedMotionError(
                    MotionErrorCode.SOFT_LIMIT,
                    f"axis {target.axis.value} velocity {velocity} mm/s exceeds "
                    f"Host limit {maximum_velocity} mm/s",
                    axis=target.axis,
                )
            assert acceleration is not None
            if acceleration > maximum_acceleration:
                raise UnifiedMotionError(
                    MotionErrorCode.SOFT_LIMIT,
                    f"axis {target.axis.value} acceleration {acceleration} mm/s² "
                    f"exceeds Host limit {maximum_acceleration} mm/s²",
                    axis=target.axis,
                )
        return velocity, acceleration

    def _refresh_stm32(self, record: _CommandRecord) -> MotionCommandResult:
        backend = self._require_backend(record.handle.axis)
        token = record.backend_token
        if not isinstance(token, STM32CommandSubmission) and not hasattr(token, "sequence"):
            raise UnifiedMotionError(
                MotionErrorCode.BACKEND_ERROR,
                "STM32 backend returned an invalid command submission",
                axis=record.handle.axis,
            )
        event = backend.poll_command(token)
        if event is None:
            state = self.get_state(record.handle.axis)
            if self._is_expected_stm32_homing_transient(record, state):
                status = (
                    MotionCommandStatus.MOVING
                    if state.busy is True
                    else MotionCommandStatus.ACCEPTED
                )
                return self._result(
                    record,
                    status,
                    final_position=state.current_position,
                    message=(
                        "reference homing remains in progress while the STM32 "
                        "position reference is invalid"
                    ),
                )
            if state.faulted:
                return self._finish(
                    record,
                    self._stm32_axis_fault_result(record, state, phase="in progress"),
                )
            status = (
                MotionCommandStatus.MOVING
                if state.busy is True
                else MotionCommandStatus.ACCEPTED
            )
            return self._result(
                record,
                status,
                final_position=state.current_position,
                message="command accepted; awaiting STM32 terminal event",
            )
        if event.kind == "DONE":
            state = self.get_state(record.handle.axis)
            result = self._validate_stm32_done_state(record, state)
            return self._finish(record, result)
        if event.kind == "ABORT":
            return self._finish(
                record,
                self._result(
                    record,
                    MotionCommandStatus.ABORTED,
                    error_code=MotionErrorCode.BACKEND_ERROR,
                    message="STM32 reported ABORT",
                ),
            )
        if event.kind == "FAULT":
            return self._finish(
                record,
                self._result(
                    record,
                    MotionCommandStatus.FAULT,
                    error_code=MotionErrorCode.DEVICE_FAULT,
                    message="STM32 reported FAULT",
                ),
            )
        raise UnifiedMotionError(
            MotionErrorCode.BACKEND_ERROR,
            f"unexpected STM32 terminal event {event.kind}",
            axis=record.handle.axis,
        )

    @staticmethod
    def _is_expected_stm32_homing_transient(
        record: _CommandRecord,
        state: AxisState,
    ) -> bool:
        """Accept POSITION_INVALID only before a home terminal event arrives."""

        return (
            record.is_home
            and state.fault_code == int(STM32AxisFault.POSITION_INVALID)
            and state.homed is False
            and state.position_valid is False
        )

    def _stm32_axis_fault_result(
        self,
        record: _CommandRecord,
        state: AxisState,
        *,
        phase: str,
    ) -> MotionCommandResult:
        raw_code = state.fault_code
        try:
            fault = STM32AxisFault(raw_code)
        except (TypeError, ValueError):
            fault = None

        name = _STM32_AXIS_FAULT_NAMES.get(fault, "stm32_axis.unknown")
        error_code = (
            MotionErrorCode.POSITION_INVALID
            if fault is STM32AxisFault.POSITION_INVALID
            else MotionErrorCode.DEVICE_FAULT
        )
        operation = "home_reference" if record.is_home else "move_absolute"
        return self._result(
            record,
            MotionCommandStatus.FAULT,
            final_position=state.current_position,
            error_code=error_code,
            message=(
                f"{name}: STM32 axis {record.handle.axis.value} reported "
                f"fault_code={raw_code!r} while {operation} was {phase}"
            ),
        )

    def _validate_stm32_done_state(
        self,
        record: _CommandRecord,
        state: AxisState,
    ) -> MotionCommandResult:
        if state.faulted:
            return self._stm32_axis_fault_result(record, state, phase="completing")
        if record.is_home and state.homed is not True:
            return self._result(
                record,
                MotionCommandStatus.FAULT,
                final_position=state.current_position,
                error_code=MotionErrorCode.POSITION_INVALID,
                message=(
                    f"STM32 axis {record.handle.axis.value} reported DONE, "
                    "but homed is not true"
                ),
            )
        if not state.position_valid:
            return self._result(
                record,
                MotionCommandStatus.FAULT,
                final_position=state.current_position,
                error_code=MotionErrorCode.POSITION_INVALID,
                message=(
                    f"STM32 axis {record.handle.axis.value} reported DONE, "
                    "but position remains invalid"
                ),
            )
        if state.busy is not False:
            return self._result(
                record,
                MotionCommandStatus.FAULT,
                final_position=state.current_position,
                error_code=MotionErrorCode.BACKEND_ERROR,
                message=(
                    f"STM32 axis {record.handle.axis.value} reported DONE, "
                    "but the axis is still busy"
                ),
            )
        return self._result(
            record,
            MotionCommandStatus.ARRIVED,
            final_position=state.current_position,
            message=(
                "reference homing completed and final axis state is homed, valid, "
                "idle, and fault-free"
                if record.is_home
                else "STM32 DONE confirmed by valid idle axis state"
            ),
        )

    def _refresh_position_axis(self, record: _CommandRecord) -> MotionCommandResult:
        state = self.get_state(record.handle.axis)
        if state.faulted:
            result = self._result(
                record,
                MotionCommandStatus.FAULT,
                final_position=state.current_position,
                error_code=MotionErrorCode.DEVICE_FAULT,
                message=state.fault_message or "device fault",
            )
            record.stop_result = self.stop(record.handle.axis)
            return self._finish(record, result)
        if not state.position_valid or state.current_position is None:
            return self._result(
                record,
                MotionCommandStatus.MOVING,
                message="position is not yet valid for arrival confirmation",
            )
        error = abs(state.current_position - record.target_position)
        config = self._arrival_configs[record.handle.axis]
        if error > config.position_tolerance:
            record.stable_since = None
            return self._result(
                record,
                MotionCommandStatus.MOVING,
                final_position=state.current_position,
                message="axis is outside the arrival tolerance",
            )
        if state.busy is not False:
            record.stable_since = None
            return self._result(
                record,
                MotionCommandStatus.MOVING,
                final_position=state.current_position,
                message="axis stationary state is not confirmed false",
            )
        now = self._clock()
        if record.stable_since is None:
            record.stable_since = now
        if now - record.stable_since < config.stable_time_s:
            return self._result(
                record,
                MotionCommandStatus.MOVING,
                final_position=state.current_position,
                message="axis is within tolerance; stable window is accumulating",
            )
        return self._finish(
            record,
            self._result(
                record,
                MotionCommandStatus.ARRIVED,
                final_position=state.current_position,
                message=(
                    "position remained within tolerance and stationary for the "
                    "stable window"
                ),
            ),
        )

    def _timeout_record(
        self,
        handle: MotionCommandHandle,
        *,
        stop_result: MotionCommandResult | None = None,
    ) -> MotionCommandResult:
        with self._lock:
            record = self._get_record(handle)
            if record.terminal_result is not None:
                return record.terminal_result
            if stop_result is None:
                stop_result = self.stop(handle.axis)
            record.stop_result = stop_result
            stop_message = (
                f"stop result {stop_result.status.value}: {stop_result.message}"
            )
            current_position = None
            try:
                current_position = self.get_state(handle.axis).current_position
            except UnifiedMotionError:
                pass
            result = self._result(
                record,
                MotionCommandStatus.TIMEOUT,
                final_position=current_position,
                error_code=MotionErrorCode.TIMEOUT,
                message=f"arrival timeout; {stop_message}",
            )
            return self._finish(record, result)

    def _group_submission_failure(
        self,
        group_id: str,
        targets: tuple[AxisTarget, ...],
        submitted: list[MotionCommandHandle],
        error: UnifiedMotionError,
    ) -> MultiAxisCommandResult:
        results: list[MotionCommandResult] = []
        submitted_by_axis = {item.axis: item for item in submitted}
        failed_axis = error.axis or targets[len(submitted)].axis
        stop_report = self.stop_axes(
            [*(item.axis for item in submitted), failed_axis]
        )
        stop_by_axis = {item.axis: item for item in stop_report.results}
        for target in targets:
            handle = submitted_by_axis.get(target.axis)
            if handle is not None:
                record = self._get_record(handle)
                stop_result = stop_by_axis[target.axis]
                result = self._result(
                    record,
                    MotionCommandStatus.ABORTED,
                    error_code=MotionErrorCode.BACKEND_ERROR,
                    message=(
                        "group submission failed; coordinated stop result: "
                        f"{stop_result.status.value}: {stop_result.message}"
                    ),
                )
                results.append(self._finish(record, result))
            else:
                is_failed = target.axis == failed_axis
                results.append(
                    MotionCommandResult(
                        command_id=uuid4().hex,
                        axis=target.axis,
                        status=MotionCommandStatus.REJECTED,
                        accepted=False,
                        completed=False,
                        target_position=target.position,
                        final_position=None,
                        position_error=None,
                        error_code=(
                            error.error_code
                            if is_failed
                            else MotionErrorCode.BACKEND_ERROR
                        ),
                        message=(
                            str(error)
                            if is_failed
                            else "not submitted after an earlier axis failed"
                        ),
                    )
                )
        return MultiAxisCommandResult(
            group_id=group_id,
            status=MotionCommandStatus.REJECTED,
            results=tuple(results),
            accepted=False,
            completed=False,
            message=(
                f"axis {failed_axis.value} submission failed; already submitted "
                "and uncertain axes received one coordinated stop attempt"
            ),
            stop_report=stop_report,
        )

    def _abort_group_peers(
        self,
        handle: MultiAxisCommandHandle,
        failed_axis: AxisName,
        stop_report: StopReport,
    ) -> None:
        stop_by_axis = {item.axis: item for item in stop_report.results}
        with self._lock:
            for command in handle.commands:
                if command.axis == failed_axis:
                    continue
                record = self._get_record(command)
                if record.terminal_result is not None:
                    continue
                if not self._descriptors[command.axis].capabilities.stop:
                    continue
                stop_result = stop_by_axis.get(command.axis)
                message = (
                    "no stop result was recorded"
                    if stop_result is None
                    else f"stop result {stop_result.status.value}: {stop_result.message}"
                )
                result = self._result(
                    record,
                    MotionCommandStatus.ABORTED,
                    error_code=MotionErrorCode.BACKEND_ERROR,
                    message=f"peer axis failed; {message}",
                )
                self._finish(record, result)

    def _best_effort_stop(self, axis: AxisName) -> str:
        if not self._descriptors[axis].capabilities.stop:
            if axis is AxisName.ROTATION:
                return "no verified independent stop is available for Rotation"
            return "backend has no independent stop capability"
        if self._backends[axis] is None:
            return "backend is unavailable, so stop could not be sent"
        try:
            result = self.stop(axis)
        except Exception as exc:
            return f"best-effort software stop failed: {exc}"
        return f"stop result {result.status.value}: {result.message}"

    def _result(
        self,
        record: _CommandRecord,
        status: MotionCommandStatus,
        *,
        final_position: float | None = None,
        error_code: MotionErrorCode | None = None,
        message: str,
    ) -> MotionCommandResult:
        record.status = status
        position_error = (
            abs(final_position - record.target_position)
            if final_position is not None
            else None
        )
        accepted, completed = {
            MotionCommandStatus.ACCEPTED: (True, None),
            MotionCommandStatus.MOVING: (True, None),
            MotionCommandStatus.ARRIVED: (True, True),
            MotionCommandStatus.REJECTED: (False, False),
            MotionCommandStatus.ABORTED: (True, False),
            MotionCommandStatus.TIMEOUT: (True, False),
            MotionCommandStatus.FAULT: (True, False),
            MotionCommandStatus.COMMUNICATION_ERROR: (True, False),
        }[status]
        return MotionCommandResult(
            command_id=record.handle.command_id,
            axis=record.handle.axis,
            status=status,
            accepted=accepted,
            completed=completed,
            target_position=record.target_position,
            final_position=final_position,
            position_error=position_error,
            error_code=error_code,
            message=message,
        )

    def _finish(
        self,
        record: _CommandRecord,
        result: MotionCommandResult,
    ) -> MotionCommandResult:
        record.status = result.status
        record.terminal_result = result
        if self._active_by_axis.get(record.handle.axis) == record.handle.command_id:
            self._active_by_axis.pop(record.handle.axis, None)
        return result

    def _ensure_axis_idle(self, axis: AxisName) -> None:
        command_id = self._active_by_axis.get(axis)
        if command_id is not None:
            raise UnifiedMotionError(
                MotionErrorCode.BUSY,
                f"axis {axis.value} already has unfinished command {command_id}",
                axis=axis,
            )

    def _get_record(self, handle: MotionCommandHandle) -> _CommandRecord:
        if not isinstance(handle, MotionCommandHandle):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "handle must be a MotionCommandHandle",
            )
        record = self._records.get(handle.command_id)
        if record is None or record.handle != handle:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                f"unknown or mismatched command handle {handle.command_id}",
                axis=handle.axis,
            )
        return record

    def _require_backend(self, axis: AxisName) -> object:
        backend = self._backends[axis]
        if backend is None:
            raise UnifiedMotionError(
                MotionErrorCode.BACKEND_UNAVAILABLE,
                f"axis {axis.value} backend is unavailable",
                axis=axis,
            )
        return backend

    def _require_suction(self) -> object:
        if self._suction is None:
            raise UnifiedMotionError(
                MotionErrorCode.BACKEND_UNAVAILABLE,
                "suction capability is unavailable",
            )
        return self._suction

    def _require_rotary_backends(self) -> dict[AxisName, object]:
        result: dict[AxisName, object] = {}
        for axis in _ROTARY_AXES:
            backend = self._backends[axis]
            if backend is None:
                raise UnifiedMotionError(
                    MotionErrorCode.BACKEND_UNAVAILABLE,
                    f"rotary joint {axis.value} backend is unavailable",
                    axis=axis,
                )
            result[axis] = backend
        return result

    @staticmethod
    def _joint_enabled(backend: object) -> bool:
        method = getattr(backend, "is_enabled", None)
        if not callable(method):
            raise JointError("CAN joint backend has no enable-state capability")
        return bool(method())

    @staticmethod
    def _rotation_enabled(backend: object) -> bool:
        method = getattr(backend, "torque_enabled", None)
        if not callable(method):
            raise FeetechRotationError(
                "Rotation backend has no torque-enable state capability"
            )
        return bool(method())

    def _require_rotary_motion_enabled(self) -> None:
        if not self.rotary_joints_enabled():
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_STATE,
                'Rotary joints are disabled. Run "joints enable" before motion.',
            )

    @staticmethod
    def _require_axis(axis: AxisName) -> AxisName:
        if not isinstance(axis, AxisName):
            raise UnifiedMotionError(
                MotionErrorCode.UNKNOWN_AXIS,
                f"unknown axis {axis!r}",
            )
        return axis

    @staticmethod
    def _validated_timeout(value: float | None, default: float) -> float:
        timeout = default if value is None else value
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "timeout must be a finite positive number",
            )
        if not math.isfinite(timeout) or timeout <= 0:
            raise UnifiedMotionError(
                MotionErrorCode.INVALID_REQUEST,
                "timeout must be finite and positive",
            )
        return float(timeout)

    @staticmethod
    def _validate_linear_position_limits(
        limits: Mapping[AxisName, tuple[float, float]],
    ) -> dict[AxisName, tuple[float, float]]:
        result = dict(limits)
        if set(result) != set(_LINEAR_AXES):
            raise ValueError("linear_position_limits must contain slide and z")
        validated: dict[AxisName, tuple[float, float]] = {}
        for axis, values in result.items():
            if not isinstance(values, tuple) or len(values) != 2:
                raise TypeError(
                    f"linear_position_limits[{axis.value}] must be a two-item tuple"
                )
            minimum, maximum = values
            for value in values:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise TypeError("linear position limits must be real numbers")
                if not math.isfinite(value):
                    raise ValueError("linear position limits must be finite")
            if minimum >= maximum:
                raise ValueError(
                    f"linear position minimum must be below maximum for {axis.value}"
                )
            validated[axis] = (float(minimum), float(maximum))
        return validated

    @staticmethod
    def _validate_linear_motion_limits(
        limits: Mapping[AxisName, tuple[float, float]],
    ) -> dict[AxisName, tuple[float, float]]:
        result = dict(limits)
        if set(result) != set(_LINEAR_AXES):
            raise ValueError("linear_motion_limits must contain slide and z")
        validated: dict[AxisName, tuple[float, float]] = {}
        for axis, values in result.items():
            if not isinstance(values, tuple) or len(values) != 2:
                raise TypeError(
                    f"linear_motion_limits[{axis.value}] must be a two-item tuple"
                )
            for value in values:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise TypeError("linear motion limits must be real numbers")
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(
                        "linear motion limits must be finite and greater than zero"
                    )
            validated[axis] = (float(values[0]), float(values[1]))
        return validated

    @staticmethod
    def _validate_arrival_configs(
        configs: Mapping[AxisName, ArrivalConfig],
    ) -> dict[AxisName, ArrivalConfig]:
        result = dict(configs)
        if set(result) != set(_AXIS_ORDER):
            raise ValueError("arrival_configs must contain exactly all five axes")
        if not all(isinstance(value, ArrivalConfig) for value in result.values()):
            raise ValueError("arrival_configs values must be ArrivalConfig")
        return result

    def _build_descriptors(
        self,
        overrides: Mapping[AxisName, AxisDescriptor] | None,
    ) -> dict[AxisName, AxisDescriptor]:
        shoulder_config = getattr(
            self._backends[AxisName.SHOULDER],
            "config",
            SHOULDER_JOINT_CONFIG,
        )
        elbow_config = getattr(
            self._backends[AxisName.ELBOW],
            "config",
            ELBOW_JOINT_CONFIG,
        )
        rotation_config = getattr(
            self._backends[AxisName.ROTATION],
            "config",
            END_EFFECTOR_ROTATION_CONFIG,
        )
        linear_capabilities = AxisCapabilities(True, True, True, True, True, True, True)
        can_capabilities = AxisCapabilities(True, True, True, False, True, False, True)
        rotation_capabilities = AxisCapabilities(
            True, True, True, False, False, False, True
        )
        result = {
            AxisName.SLIDE: AxisDescriptor(
                AxisName.SLIDE,
                "Slide",
                AxisKind.LINEAR,
                "mm",
                "mm/s",
                "mm/s²",
                *self._linear_position_limits[AxisName.SLIDE],
                linear_capabilities,
            ),
            AxisName.Z: AxisDescriptor(
                AxisName.Z,
                "Z",
                AxisKind.LINEAR,
                "mm",
                "mm/s",
                "mm/s²",
                *self._linear_position_limits[AxisName.Z],
                linear_capabilities,
            ),
            AxisName.SHOULDER: AxisDescriptor(
                AxisName.SHOULDER,
                "Shoulder",
                AxisKind.ROTARY,
                "deg",
                "deg/s",
                "deg/s²",
                math.degrees(shoulder_config.min_position_rad),
                math.degrees(shoulder_config.max_position_rad),
                can_capabilities,
            ),
            AxisName.ELBOW: AxisDescriptor(
                AxisName.ELBOW,
                "Elbow",
                AxisKind.ROTARY,
                "deg",
                "deg/s",
                "deg/s²",
                math.degrees(elbow_config.min_position_rad),
                math.degrees(elbow_config.max_position_rad),
                can_capabilities,
            ),
            AxisName.ROTATION: AxisDescriptor(
                AxisName.ROTATION,
                "End-effector rotation",
                AxisKind.ROTARY,
                "deg",
                "deg/s",
                "deg/s²",
                math.degrees(rotation_config.min_position_rad),
                math.degrees(rotation_config.max_position_rad),
                rotation_capabilities,
            ),
        }
        for axis, descriptor in dict(overrides or {}).items():
            if not isinstance(axis, AxisName) or descriptor.name != axis:
                raise ValueError("axis_descriptors keys must match descriptor names")
            result[axis] = descriptor
        return result

    def _submission_error(
        self,
        axis: AxisName,
        exc: Exception,
    ) -> UnifiedMotionError:
        if isinstance(exc, UnifiedMotionError):
            if exc.axis is not None:
                return exc
            return UnifiedMotionError(exc.error_code, str(exc), axis=axis)
        return UnifiedMotionError(self._map_exception(exc), str(exc), axis=axis)

    @staticmethod
    def _map_exception(exc: Exception) -> MotionErrorCode:
        if isinstance(exc, STM32CommandError):
            return {
                4: MotionErrorCode.INVALID_REQUEST,
                5: MotionErrorCode.BUSY,
                7: MotionErrorCode.NOT_HOMED,
                8: MotionErrorCode.SOFT_LIMIT,
                9: MotionErrorCode.DEVICE_FAULT,
                12: MotionErrorCode.UNSUPPORTED_COMMAND,
                13: MotionErrorCode.BUSY,
                14: MotionErrorCode.DEVICE_FAULT,
            }.get(exc.error_code, MotionErrorCode.BACKEND_ERROR)
        if isinstance(exc, JointLimitError | FeetechRotationLimitError):
            return MotionErrorCode.SOFT_LIMIT
        if isinstance(exc, JointInitializationError):
            return MotionErrorCode.POSITION_INVALID
        if isinstance(exc, JointPositionOutOfRangeError | FeetechRotationPositionError):
            return MotionErrorCode.POSITION_INVALID
        if isinstance(exc, JointMotorMovingError):
            return MotionErrorCode.BUSY
        if isinstance(exc, JointMotorFaultError | FeetechDeviceError):
            return MotionErrorCode.DEVICE_FAULT
        if isinstance(exc, JointMotorDisabledError):
            return MotionErrorCode.POSITION_INVALID
        if isinstance(exc, FeetechNotOpenError):
            return MotionErrorCode.BACKEND_UNAVAILABLE
        if isinstance(exc, CanBusNotOpenError):
            return MotionErrorCode.BACKEND_UNAVAILABLE
        if isinstance(exc, MotorCommunicationError):
            return MotionErrorCode.COMMUNICATION_ERROR
        if isinstance(
            exc,
            (
                STM32MotionTimeoutError,
                STM32MotionProtocolError,
                FeetechTimeoutError,
                FeetechProtocolError,
            ),
        ):
            return MotionErrorCode.COMMUNICATION_ERROR
        if isinstance(exc, STM32MotionConfigurationError):
            return MotionErrorCode.INVALID_REQUEST
        if isinstance(exc, (STM32MotionError, FeetechError)):
            return MotionErrorCode.COMMUNICATION_ERROR
        if isinstance(exc, (JointError, FeetechRotationError)):
            return MotionErrorCode.BACKEND_ERROR
        return MotionErrorCode.BACKEND_ERROR

    @staticmethod
    def _status_for_error(error_code: MotionErrorCode) -> MotionCommandStatus:
        if error_code in (
            MotionErrorCode.COMMUNICATION_ERROR,
            MotionErrorCode.BACKEND_UNAVAILABLE,
        ):
            return MotionCommandStatus.COMMUNICATION_ERROR
        return MotionCommandStatus.FAULT

    def _prune_records(self) -> None:
        overflow = len(self._records) - self._max_command_history
        if overflow <= 0:
            return
        for command_id, record in tuple(self._records.items()):
            if overflow <= 0:
                break
            if record.terminal_result is not None:
                self._records.pop(command_id, None)
                overflow -= 1


def _millimetres_to_micrometres(
    value: float,
    name: str,
    *,
    unsigned: bool = False,
) -> int:
    converted = round(value * 1000.0)
    minimum, maximum = (1, _UINT32_MAX) if unsigned else (_INT32_MIN, _INT32_MAX)
    if not minimum <= converted <= maximum:
        raise UnifiedMotionError(
            MotionErrorCode.INVALID_REQUEST,
            f"{name} converts to {converted} µm outside backend integer range",
        )
    return converted


def _micrometres_to_millimetres(value: int) -> float:
    return value / 1000.0


def _stm32_axis_fault_name(raw_code: int) -> str:
    try:
        fault = STM32AxisFault(raw_code)
    except ValueError:
        return "stm32_axis.unknown"
    return _STM32_AXIS_FAULT_NAMES.get(fault, "stm32_axis.unknown")


__all__ = [
    "MultiAxisSubmissionError",
    "UnifiedMotionController",
    "UnifiedMotionError",
]
