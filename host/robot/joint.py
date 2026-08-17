"""基于 MG4010 单圈绝对编码器的有限行程旋转关节。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from drivers.mg4010_driver import MG4010Driver
from drivers.mg4010_protocol import MotorFault, MotorSingleTurnPosition, MotorStatus


_ANGLE_EPSILON_RAD = 1e-12
_DEFAULT_INITIALIZATION_SAMPLES = 3
_DEFAULT_INITIALIZATION_ATTEMPTS = 12
_DEFAULT_INITIALIZATION_STABILITY_DEG = 0.1
_DEFAULT_INITIALIZATION_SAMPLE_INTERVAL = 0.02


class JointError(Exception):
    """旋转关节层基础异常。"""


class JointConfigurationError(JointError):
    """关节配置不完整、矛盾或产生歧义。"""


class JointInitializationError(JointError):
    """关节尚未完成稳定的绝对位置初始化。"""


class JointLimitError(JointError):
    """关节运动目标或速度超过软件限制。"""


class JointPositionOutOfRangeError(JointError):
    """绝对编码器位置无法映射到关节有限行程。"""


class JointMotorFaultError(JointError):
    """电机报告非零错误状态。"""


class JointMotorDisabledError(JointError):
    """电机处于关闭或未知运行状态。"""


class JointMotorMovingError(JointError):
    """电机仍在运动，无法安全建立一致的位置命令快照。"""


class JointEnableStateError(JointError):
    """MG4010 使能/失能命令后的 0x9A 状态与目标不一致。"""


@dataclass(frozen=True)
class JointConfig:
    """已标定有限行程旋转关节的不可变配置。

    ``encoder_zero_output_deg`` 是逻辑关节 0 rad 对应的输出轴绝对角；
    ``min_position_rad``、``max_position_rad`` 和 ``max_velocity_rad_s``
    均属于逻辑关节语义，不是电机侧或输出轴绝对角限位。
    """

    name: str
    motor_id: int
    gear_ratio: float
    direction_sign: int
    encoder_zero_output_deg: float
    min_position_rad: float
    max_position_rad: float
    max_velocity_rad_s: float
    position_tolerance_rad: float
    moving_velocity_threshold_rad_s: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise JointConfigurationError("joint name must not be empty")
        if not 1 <= self.motor_id <= 32:
            raise JointConfigurationError(
                f"joint {self.name}: motor_id must be in 1..32, got {self.motor_id}"
            )
        numeric_values = {
            "gear_ratio": self.gear_ratio,
            "encoder_zero_output_deg": self.encoder_zero_output_deg,
            "min_position_rad": self.min_position_rad,
            "max_position_rad": self.max_position_rad,
            "max_velocity_rad_s": self.max_velocity_rad_s,
            "position_tolerance_rad": self.position_tolerance_rad,
            "moving_velocity_threshold_rad_s": self.moving_velocity_threshold_rad_s,
        }
        for field_name, value in numeric_values.items():
            if not math.isfinite(value):
                raise JointConfigurationError(
                    f"joint {self.name}: {field_name} must be finite, got {value!r}"
                )
        if self.gear_ratio <= 0:
            raise JointConfigurationError(
                f"joint {self.name}: gear_ratio must be greater than zero"
            )
        if self.direction_sign not in (-1, 1):
            raise JointConfigurationError(
                f"joint {self.name}: direction_sign must be -1 or +1"
            )
        if self.min_position_rad >= self.max_position_rad:
            raise JointConfigurationError(
                f"joint {self.name}: min_position_rad must be less than "
                "max_position_rad"
            )
        if self.max_position_rad - self.min_position_rad >= math.tau:
            raise JointConfigurationError(
                f"joint {self.name}: finite travel width must be less than 2*pi rad"
            )
        for field_name in (
            "max_velocity_rad_s",
            "position_tolerance_rad",
            "moving_velocity_threshold_rad_s",
        ):
            if getattr(self, field_name) <= 0:
                raise JointConfigurationError(
                    f"joint {self.name}: {field_name} must be greater than zero"
                )


@dataclass(frozen=True)
class JointState:
    """一次关节状态快照；协议未读取的字段保持为 ``None``。"""

    timestamp_monotonic: float
    circle_angle_raw: int
    motor_cycle_deg: float
    output_abs_deg: float
    position_rad: float
    motor_multi_turn_deg: float | None
    motor_speed_deg_s: float | None
    velocity_rad_s: float | None
    temperature_c: int | None
    motor_state: int | None
    error_state: int | None
    position_valid: bool
    moving: bool


@dataclass(frozen=True)
class JointHoldSnapshot:
    """已提交的当前位置保持命令所依据的最后一组电机快照。"""

    target_position_rad: float
    target_motor_multi_turn_deg: float
    sampled_motor_speed_deg_s: float
    commanded_max_motor_speed_deg_s: float


@dataclass(frozen=True)
class PreparedJointPositionCommand:
    """已完成安全预检、等待立即提交的一次性 A4 位置命令。"""

    joint_name: str
    motor_id: int
    preparation_id: int
    state: JointState
    target_motor_multi_turn_deg: float | None
    max_motor_speed_deg_s: float | None

    def __post_init__(self) -> None:
        target = self.target_motor_multi_turn_deg
        speed = self.max_motor_speed_deg_s
        if (target is None) != (speed is None):
            raise ValueError(
                "prepared position target and speed must both be set or both be None"
            )
        if target is not None:
            assert speed is not None
            if not math.isfinite(target) or not math.isfinite(speed) or speed <= 0:
                raise ValueError(
                    "prepared position target and speed must be finite and positive"
                )

    @property
    def command_required(self) -> bool:
        return self.target_motor_multi_turn_deg is not None


def wrap_360(angle_deg: float) -> float:
    """将有限角度归一化到 ``[0, 360)``。"""

    if not math.isfinite(angle_deg):
        raise ValueError(f"angle_deg must be finite, got {angle_deg!r}")
    wrapped = angle_deg % 360.0
    return 0.0 if math.isclose(wrapped, 360.0, abs_tol=1e-12) else wrapped


def resolve_output_angle_to_joint_position(
    output_abs_deg: float,
    config: JointConfig,
) -> float:
    """把 0x94 输出轴绝对角解析为软限位内唯一的逻辑关节角。"""

    output_abs_deg = wrap_360(output_abs_deg)
    zero_deg = wrap_360(config.encoder_zero_output_deg)
    candidates: list[float] = []
    for turns in range(-2, 3):
        output_delta_deg = output_abs_deg - zero_deg + 360.0 * turns
        candidate_rad = config.direction_sign * math.radians(output_delta_deg)
        if (
            config.min_position_rad - _ANGLE_EPSILON_RAD
            <= candidate_rad
            <= config.max_position_rad + _ANGLE_EPSILON_RAD
        ):
            candidates.append(candidate_rad)

    if not candidates:
        raise JointPositionOutOfRangeError(
            f"joint {config.name} motor ID {config.motor_id}: output absolute angle "
            f"{output_abs_deg:.6f} deg cannot map into "
            f"[{config.min_position_rad:.6f}, {config.max_position_rad:.6f}] rad"
        )
    if len(candidates) != 1:
        rendered = ", ".join(f"{value:.12f}" for value in candidates)
        raise JointConfigurationError(
            f"joint {config.name} motor ID {config.motor_id}: absolute angle "
            f"{output_abs_deg:.6f} deg has multiple legal candidates: {rendered}"
        )
    return min(max(candidates[0], config.min_position_rad), config.max_position_rad)


def joint_position_to_output_abs_deg(
    joint_position_rad: float,
    config: JointConfig,
) -> float:
    """把逻辑关节角转换为等效的输出轴单圈绝对角。"""

    _validate_finite_joint_position(joint_position_rad, config)
    return wrap_360(
        config.encoder_zero_output_deg
        + config.direction_sign * math.degrees(joint_position_rad)
    )


def joint_velocity_to_motor_speed_deg_s(
    velocity_rad_s: float,
    config: JointConfig,
) -> float:
    """按当前协议解释把关节速度限制换算为 A4 电机侧 dps。"""

    if not math.isfinite(velocity_rad_s) or velocity_rad_s <= 0:
        raise JointLimitError(
            f"joint {config.name}: velocity must be finite and positive, "
            f"got {velocity_rad_s!r}"
        )
    if velocity_rad_s > config.max_velocity_rad_s:
        raise JointLimitError(
            f"joint {config.name}: velocity {velocity_rad_s:.6f} rad/s exceeds "
            f"limit {config.max_velocity_rad_s:.6f} rad/s"
        )
    # V2.36 将 A4 maxSpeed 定义为电机侧 1 dps/LSB；MG4010E-i36
    # 的实测结果与乘以减速比一致。如厂商后续澄清，只需集中修改此函数。
    motor_speed = math.degrees(velocity_rad_s) * config.gear_ratio
    max_speed_raw = round(motor_speed)
    if not 1 <= max_speed_raw <= 0xFFFF:
        raise JointLimitError(
            f"joint {config.name}: converted motor speed {motor_speed:.6f} deg/s "
            "does not fit A4 uint16 maxSpeed"
        )
    return motor_speed


def _circular_difference_deg(first: float, second: float) -> float:
    return (first - second + 180.0) % 360.0 - 180.0


def _validate_finite_joint_position(position_rad: float, config: JointConfig) -> None:
    if not math.isfinite(position_rad):
        raise JointLimitError(
            f"joint {config.name}: target position must be finite, got {position_rad!r}"
        )


class CanRotaryJoint:
    """把 MG4010 电机侧协议映射为有限行程弧度制关节接口。"""

    def __init__(self, driver: MG4010Driver, config: JointConfig) -> None:
        if not isinstance(config, JointConfig):
            raise JointConfigurationError(
                "a fully calibrated JointConfig is required; unconfigured "
                "shoulder/elbow templates cannot be used for motion"
            )
        if driver.motor_id != config.motor_id:
            raise JointConfigurationError(
                f"joint {config.name}: driver motor ID {driver.motor_id} does not "
                f"match config motor ID {config.motor_id}"
            )
        self.driver = driver
        self.config = config
        self._initialized = False
        self._last_state: JointState | None = None
        self._next_preparation_id = 1
        self._pending_preparation_id: int | None = None

    def initialize(
        self,
        *,
        stable_samples: int = _DEFAULT_INITIALIZATION_SAMPLES,
        max_attempts: int = _DEFAULT_INITIALIZATION_ATTEMPTS,
        stability_tolerance_deg: float = _DEFAULT_INITIALIZATION_STABILITY_DEG,
        sample_interval: float = _DEFAULT_INITIALIZATION_SAMPLE_INTERVAL,
    ) -> JointState:
        """用至少三次稳定的 0x94 只读样本建立绝对关节位置。"""

        self._pending_preparation_id = None
        if stable_samples < 3:
            raise ValueError("stable_samples must be at least 3")
        if max_attempts < stable_samples:
            raise ValueError("max_attempts must be at least stable_samples")
        if not math.isfinite(stability_tolerance_deg) or stability_tolerance_deg <= 0:
            raise ValueError("stability_tolerance_deg must be finite and positive")
        if not math.isfinite(sample_interval) or sample_interval < 0:
            raise ValueError("sample_interval must be finite and non-negative")

        previous_output_deg: float | None = None
        consecutive = 0
        last_single: MotorSingleTurnPosition | None = None
        for _attempt in range(max_attempts):
            single = self.driver.read_single_turn_position()
            output_abs_deg = self._output_abs_deg(single)
            if previous_output_deg is None or abs(
                _circular_difference_deg(output_abs_deg, previous_output_deg)
            ) <= stability_tolerance_deg:
                consecutive += 1
            else:
                consecutive = 1
            previous_output_deg = output_abs_deg
            last_single = single
            if consecutive >= stable_samples:
                position_rad = resolve_output_angle_to_joint_position(
                    output_abs_deg, self.config
                )
                state = self._compose_state(
                    single=single,
                    position_rad=position_rad,
                )
                self._initialized = True
                self._last_state = state
                return state
            if sample_interval > 0:
                time.sleep(sample_interval)

        raise JointInitializationError(
            f"joint {self.config.name} motor ID {self.config.motor_id}: 0x94 did not "
            f"produce {stable_samples} stable samples in {max_attempts} attempts; "
            f"last sample={last_single!r}"
        )

    def get_position(self) -> float:
        """读取并返回当前逻辑关节位置，单位 rad。"""

        return self.get_state().position_rad

    def get_velocity(self) -> float | None:
        """读取并返回当前逻辑关节速度，单位 rad/s。"""

        return self.get_state().velocity_rad_s

    def get_state(self) -> JointState:
        """同步读取 0x94、0x92、0x9C 和 0x9A，返回状态快照。"""

        self._require_initialized()
        single = self.driver.read_single_turn_position()
        output_abs_deg = self._output_abs_deg(single)
        position_rad = resolve_output_angle_to_joint_position(
            output_abs_deg, self.config
        )
        multi_turn_deg = self.driver.read_multi_turn_position_deg()
        status = self.driver.read_status()
        fault = self.driver.read_fault()
        state = self._compose_state(
            single=single,
            position_rad=position_rad,
            multi_turn_deg=multi_turn_deg,
            status=status,
            fault=fault,
        )
        self._last_state = state
        return state

    def get_fault(self) -> int | None:
        """读取并返回原始错误状态字节。"""

        return self.driver.read_fault().error_state

    def is_moving(self) -> bool:
        """按配置的关节速度阈值判断电机是否在运动。"""

        return self.get_state().moving

    def is_enabled(self) -> bool:
        """通过 0x9A 读取协议定义的实时运行状态。"""

        state = self.driver.read_fault().motor_state
        if state == 0x00:
            return True
        if state == 0x10:
            return False
        raise JointEnableStateError(
            f"joint {self.config.name} motor ID {self.config.motor_id}: "
            f"unknown motor state 0x{state:02X}; expected 0x00 or 0x10"
        )

    def enable(self) -> None:
        """发送 0x88，并通过 0x9A 确认保持使能。"""

        self._pending_preparation_id = None
        self.driver.enable()
        if not self.is_enabled():
            raise JointEnableStateError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                "0x88 was acknowledged but motor state remains disabled"
            )

    def disable(self) -> None:
        """发送 0x80，并通过 0x9A 确认已经移除保持力。"""

        self._pending_preparation_id = None
        self.driver.disable()
        if self.is_enabled():
            raise JointEnableStateError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                "0x80 was acknowledged but motor state remains enabled"
            )

    def command_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
    ) -> JointState:
        """提交非阻塞位置目标；仅等待通信应答，不等待机械到位。"""

        prepared = self.prepare_position_command(position_rad, velocity_rad_s)
        return self.submit_prepared_position_command(prepared)

    def prepare_position_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
    ) -> PreparedJointPositionCommand:
        """完成实时安全预检和目标计算，但不发送 A4。"""

        self._pending_preparation_id = None
        self._validate_command(position_rad, velocity_rad_s)
        self._require_initialized()
        single = self.driver.read_single_turn_position()
        output_abs_deg = self._output_abs_deg(single)
        current_position_rad = resolve_output_angle_to_joint_position(
            output_abs_deg, self.config
        )
        status = self.driver.read_status()
        fault = self.driver.read_fault()
        current = self._compose_state(
            single=single,
            position_rad=current_position_rad,
            status=status,
            fault=fault,
        )
        if not current.position_valid:
            raise JointInitializationError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                "current absolute position is not valid"
            )
        if current.error_state is None or current.error_state != 0:
            rendered = "unknown" if current.error_state is None else f"0x{current.error_state:02X}"
            raise JointMotorFaultError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"motor error state is {rendered}; no position command was sent"
            )
        if current.motor_state != 0x00:
            rendered = "unknown" if current.motor_state is None else f"0x{current.motor_state:02X}"
            raise JointMotorDisabledError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"motor state {rendered} is not the protocol-defined enabled state "
                "0x00; no position command was sent"
            )
        if current.moving:
            raise JointMotorMovingError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"motor is moving at {current.velocity_rad_s:.6f} rad/s; wait "
                "for a stationary state before submitting another position command"
            )

        delta_joint_rad = position_rad - current.position_rad
        if abs(delta_joint_rad) <= self.config.position_tolerance_rad:
            return self._prepared_position_command(current)
        current_multi_turn_deg = self.driver.read_multi_turn_position_deg()
        confirmed_single = self.driver.read_single_turn_position()
        confirmed_output_abs_deg = self._output_abs_deg(confirmed_single)
        confirmed_position_rad = resolve_output_angle_to_joint_position(
            confirmed_output_abs_deg, self.config
        )
        position_sample_drift = confirmed_position_rad - current_position_rad
        if abs(position_sample_drift) > self.config.position_tolerance_rad:
            raise JointMotorMovingError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                "0x94 changed while pairing it with the current 0x92 coordinate "
                f"(drift {position_sample_drift:+.6f} rad); no position command "
                "was sent"
            )
        delta_joint_rad = position_rad - confirmed_position_rad
        if abs(delta_joint_rad) <= self.config.position_tolerance_rad:
            current = self._compose_state(
                single=confirmed_single,
                position_rad=confirmed_position_rad,
                multi_turn_deg=current_multi_turn_deg,
                status=status,
                fault=fault,
            )
            return self._prepared_position_command(current)
        current = self._compose_state(
            single=confirmed_single,
            position_rad=confirmed_position_rad,
            multi_turn_deg=current_multi_turn_deg,
            status=status,
            fault=fault,
        )

        motor_delta_deg = (
            self.config.direction_sign
            * math.degrees(delta_joint_rad)
            * self.config.gear_ratio
        )
        target_motor_multi_turn_deg = current_multi_turn_deg + motor_delta_deg
        max_motor_speed_deg_s = joint_velocity_to_motor_speed_deg_s(
            velocity_rad_s, self.config
        )
        return self._prepared_position_command(
            current,
            target_motor_multi_turn_deg=target_motor_multi_turn_deg,
            max_motor_speed_deg_s=max_motor_speed_deg_s,
        )

    def submit_prepared_position_command(
        self,
        prepared: PreparedJointPositionCommand,
    ) -> JointState:
        """立即提交本关节的一次性准备命令，不再次读取实时反馈。"""

        if not isinstance(prepared, PreparedJointPositionCommand):
            raise JointConfigurationError(
                f"joint {self.config.name}: prepared command has an invalid type"
            )
        if (
            prepared.joint_name != self.config.name
            or prepared.motor_id != self.config.motor_id
        ):
            raise JointConfigurationError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: prepared "
                f"command belongs to joint {prepared.joint_name} motor ID "
                f"{prepared.motor_id}"
            )
        if prepared.preparation_id != self._pending_preparation_id:
            raise JointConfigurationError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: prepared "
                "command is stale or has already been submitted"
            )
        self._pending_preparation_id = None
        if prepared.command_required:
            assert prepared.target_motor_multi_turn_deg is not None
            assert prepared.max_motor_speed_deg_s is not None
            self.driver.command_position(
                target_motor_deg=prepared.target_motor_multi_turn_deg,
                max_motor_speed_deg_s=prepared.max_motor_speed_deg_s,
            )
        self._last_state = prepared.state
        return prepared.state

    def _prepared_position_command(
        self,
        state: JointState,
        *,
        target_motor_multi_turn_deg: float | None = None,
        max_motor_speed_deg_s: float | None = None,
    ) -> PreparedJointPositionCommand:
        preparation_id = self._next_preparation_id
        self._next_preparation_id += 1
        self._pending_preparation_id = preparation_id
        return PreparedJointPositionCommand(
            joint_name=self.config.name,
            motor_id=self.config.motor_id,
            preparation_id=preparation_id,
            state=state,
            target_motor_multi_turn_deg=target_motor_multi_turn_deg,
            max_motor_speed_deg_s=max_motor_speed_deg_s,
        )

    def stop(self) -> JointHoldSnapshot:
        """读取当前位置并提交 A4 位置保持；绝不回退到 0x81。"""

        self._pending_preparation_id = None
        self._require_initialized()
        fault = self.driver.read_fault()
        if fault.error_state != 0:
            raise JointMotorFaultError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"motor error state is 0x{fault.error_state:02X}; no hold command was sent"
            )
        if fault.motor_state != 0x00:
            raise JointMotorDisabledError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"motor state 0x{fault.motor_state:02X} is not the protocol-defined "
                "enabled state 0x00; no hold command was sent"
            )

        status = self.driver.read_status()
        single = self.driver.read_single_turn_position()
        position_rad = resolve_output_angle_to_joint_position(
            self._output_abs_deg(single), self.config
        )
        # 0x92 必须最后读取，使 A4 目标尽量贴近发送前的电机坐标。
        motor_multi_turn_deg = self.driver.read_multi_turn_position_deg()
        configured_max = math.degrees(self.config.max_velocity_rad_s) * self.config.gear_ratio
        hold_speed = min(
            configured_max,
            max(1.0, abs(float(status.motor_speed_deg_s))),
        )
        self.driver.command_position(
            target_motor_deg=motor_multi_turn_deg,
            max_motor_speed_deg_s=hold_speed,
        )
        self._last_state = self._compose_state(
            single=single,
            position_rad=position_rad,
            multi_turn_deg=motor_multi_turn_deg,
            status=status,
            fault=fault,
        )
        return JointHoldSnapshot(
            target_position_rad=position_rad,
            target_motor_multi_turn_deg=motor_multi_turn_deg,
            sampled_motor_speed_deg_s=float(status.motor_speed_deg_s),
            commanded_max_motor_speed_deg_s=hold_speed,
        )

    def validate_position_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
    ) -> None:
        """只验证位置和速度参数，不访问 CAN，也不发送命令。"""

        self._validate_command(position_rad, velocity_rad_s)

    def _validate_command(self, position_rad: float, velocity_rad_s: float) -> None:
        _validate_finite_joint_position(position_rad, self.config)
        if not (
            self.config.min_position_rad
            <= position_rad
            <= self.config.max_position_rad
        ):
            raise JointLimitError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: target "
                f"{position_rad:.6f} rad is outside "
                f"[{self.config.min_position_rad:.6f}, "
                f"{self.config.max_position_rad:.6f}] rad"
            )
        joint_velocity_to_motor_speed_deg_s(velocity_rad_s, self.config)

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise JointInitializationError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: call "
                "initialize() and obtain stable 0x94 samples before motion"
            )

    def _compose_state(
        self,
        *,
        single: MotorSingleTurnPosition,
        position_rad: float,
        multi_turn_deg: float | None = None,
        status: MotorStatus | None = None,
        fault: MotorFault | None = None,
    ) -> JointState:
        output_abs_deg = self._output_abs_deg(single)
        motor_speed = status.motor_speed_deg_s if status is not None else None
        velocity = (
            self.config.direction_sign
            * math.radians(motor_speed / self.config.gear_ratio)
            if motor_speed is not None
            else None
        )
        return JointState(
            timestamp_monotonic=time.monotonic(),
            circle_angle_raw=single.circle_angle_raw,
            motor_cycle_deg=single.motor_cycle_deg,
            output_abs_deg=output_abs_deg,
            position_rad=position_rad,
            motor_multi_turn_deg=multi_turn_deg,
            motor_speed_deg_s=motor_speed,
            velocity_rad_s=velocity,
            temperature_c=(status.temperature_c if status is not None else None),
            motor_state=(fault.motor_state if fault is not None else None),
            error_state=(fault.error_state if fault is not None else None),
            position_valid=True,
            moving=(
                abs(velocity) > self.config.moving_velocity_threshold_rad_s
                if velocity is not None
                else False
            ),
        )

    def _output_abs_deg(self, single: MotorSingleTurnPosition) -> float:
        cycle_width_motor_deg = 360.0 * self.config.gear_ratio
        if not (
            0 <= single.circle_angle_raw < round(cycle_width_motor_deg * 100)
            and 0.0 <= single.motor_cycle_deg < cycle_width_motor_deg
        ):
            raise JointPositionOutOfRangeError(
                f"joint {self.config.name} motor ID {self.config.motor_id}: "
                f"0x94 value raw={single.circle_angle_raw}, "
                f"motor_cycle_deg={single.motor_cycle_deg!r} is outside one "
                f"configured {self.config.gear_ratio:g}:1 output cycle"
            )
        return wrap_360(single.motor_cycle_deg / self.config.gear_ratio)
