"""上层运动 Runtime 组装、生命周期、授权和只读 smoke 的离线测试。"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

from bootstrap import (
    HardwareCloseError,
    HardwareOpenError,
    UpperMotionRuntime,
    create_upper_motion_runtime,
)
from config.project.feetech import END_EFFECTOR_ROTATION_CONFIG
from config.hardware import (
    GsUsbDeviceConfig,
    HardwareConfig,
    UsbSerialDeviceConfig,
    UsbVidPid,
)
from config.project.joints import ELBOW_JOINT_CONFIG, SHOULDER_JOINT_CONFIG
from config.motion_runtime import (
    AxisMotionProfile,
    LinearAxisMotionLimits,
    LinearAxisPositionLimits,
    MotionRuntimeConfig,
)
from drivers.device_discovery import (
    ResolvedGsUsbDevice,
    ResolvedHardware,
    ResolvedSerialDevice,
)
from drivers.stm32_motion import AxisStatus
from motion.authorization import (
    MotionAuthorization,
    MotionAuthorizationError,
    RotationMotionAuthorizationError,
    RuntimeMode,
)
from motion.unified_controller import UnifiedMotionController
from motion.unified_protocol import (
    ArrivalConfig,
    AxisName,
    AxisTarget,
    MotionErrorCode,
    MultiAxisTarget,
)
from scripts.manual_motion import main as smoke_main
from scripts.manual_motion import run_inspect as run_read_only_smoke


def hardware_config() -> HardwareConfig:
    return HardwareConfig(
        feetech=UsbSerialDeviceConfig(UsbVidPid(0x1A86, 0x55D3), 115200),
        stm32_motion=UsbSerialDeviceConfig(UsbVidPid(0x0483, 0x374B), 230400),
        can_adapter=GsUsbDeviceConfig(UsbVidPid(0x1D50, 0x606F), 1_000_000),
    )


def motion_config() -> MotionRuntimeConfig:
    def profile(
        velocity: float | None,
        acceleration: float | None,
    ) -> AxisMotionProfile:
        return AxisMotionProfile(
            velocity,
            acceleration,
            ArrivalConfig(0.2, 0.0, 0.01, 1.0),
        )

    return MotionRuntimeConfig(
        slide=profile(2.0, 4.0),
        z=profile(1.0, 3.0),
        shoulder=profile(2.0, None),
        elbow=profile(2.0, None),
        rotation=profile(None, None),
        slide_position_limits=LinearAxisPositionLimits(0.0, 700.0),
        z_position_limits=LinearAxisPositionLimits(0.0, 180.0),
        slide_motion_limits=LinearAxisMotionLimits(72.0, 180.0),
        z_motion_limits=LinearAxisMotionLimits(10.0, 25.0),
    )


def resolved_hardware() -> ResolvedHardware:
    stm32 = ResolvedSerialDevice(
        "COM7", 0x0483, 0x374B, "STM32", None, None, None, None, None
    )
    feetech = ResolvedSerialDevice(
        "/dev/ttyUSB9", 0x1A86, 0x55D3, "FEETECH", None, None, None, None, None
    )
    usb_device = SimpleNamespace(bus=3, address=4)
    can_adapter = ResolvedGsUsbDevice(
        usb_device, 0x1D50, 0x606F, "CAN", 3, 4, "gs_usb"
    )
    return ResolvedHardware(stm32, can_adapter, feetech)


def lifecycle_runtime(
    stm32: Mock,
    can_bus: Mock,
    feetech: Mock,
    *,
    controller: Mock | None = None,
) -> UpperMotionRuntime:
    return UpperMotionRuntime(
        resolved_hardware=resolved_hardware(),
        hardware_config=hardware_config(),
        motion_config=motion_config(),
        authorization=Mock(),
        stm32_transport=stm32,
        stm32_client=Mock(),
        can_bus=can_bus,
        shoulder_joint=Mock(),
        elbow_joint=Mock(),
        feetech_bus=feetech,
        rotation_axis=Mock(),
        controller=controller or Mock(),
    )


class BootstrapAssemblyTests(unittest.TestCase):
    @patch("bootstrap.resolve_hardware")
    def test_factory_resolves_once_and_injects_runtime_identity(self, resolver: Mock) -> None:
        resolved = resolved_hardware()
        resolver.return_value = resolved
        hardware = hardware_config()
        motion = motion_config()

        runtime = create_upper_motion_runtime(hardware, motion)

        resolver.assert_called_once_with(hardware)
        self.assertIs(runtime.resolved_hardware, resolved)
        self.assertEqual(runtime.stm32_transport.config.port, "COM7")
        self.assertEqual(runtime.stm32_transport.config.baudrate, 230400)
        self.assertEqual(runtime.feetech_bus.config.port, "/dev/ttyUSB9")
        self.assertIs(runtime.can_bus.gs_usb_device, resolved.can_adapter.device)
        self.assertEqual(runtime.can_bus.bitrate, 1_000_000)
        self.assertTrue(runtime.can_bus.allow_same_id_response)
        self.assertIs(runtime.shoulder_joint.config, SHOULDER_JOINT_CONFIG)
        self.assertIs(runtime.elbow_joint.config, ELBOW_JOINT_CONFIG)
        self.assertIs(runtime.rotation_axis.config, END_EFFECTOR_ROTATION_CONFIG)
        self.assertIs(runtime.shoulder_joint.driver.bus, runtime.can_bus)
        self.assertIs(runtime.elbow_joint.driver.bus, runtime.can_bus)
        self.assertIs(runtime.controller._backends[AxisName.SLIDE], runtime.stm32_client)
        self.assertIs(runtime.controller._suction._client, runtime.stm32_client)
        self.assertIs(
            runtime.controller._backends[AxisName.SHOULDER],
            runtime.shoulder_joint,
        )
        self.assertIs(runtime.controller._backends[AxisName.ELBOW], runtime.elbow_joint)
        self.assertIs(
            runtime.controller._backends[AxisName.ROTATION],
            runtime.rotation_axis,
        )
        self.assertEqual(
            (
                runtime.controller.describe_axis(AxisName.SLIDE).minimum_position,
                runtime.controller.describe_axis(AxisName.SLIDE).maximum_position,
            ),
            (0.0, 700.0),
        )
        self.assertEqual(
            (
                runtime.controller.describe_axis(AxisName.Z).minimum_position,
                runtime.controller.describe_axis(AxisName.Z).maximum_position,
            ),
            (0.0, 180.0),
        )
        self.assertIs(runtime.frontend_motion._controller, runtime.controller)
        self.assertIs(runtime.kinematics_motion._controller, runtime.controller)

    @patch("bootstrap.FeetechRotationAxis.command_position")
    @patch("bootstrap.FeetechRotationAxis.enable_torque")
    @patch("bootstrap.CanRotaryJoint.command_position")
    @patch("bootstrap.CanRotaryJoint.initialize")
    @patch("bootstrap.STM32MotionClient.submit_home")
    @patch("bootstrap.STM32MotionClient.submit_move_absolute")
    @patch("bootstrap.STM32MotionClient.enable")
    @patch("bootstrap.FeetechBus.open")
    @patch("bootstrap.CanMotorBus.open")
    @patch("bootstrap.STM32SerialTransport.open")
    @patch("bootstrap.resolve_hardware")
    def test_factory_constructs_without_hardware_or_control_io(
        self,
        resolver: Mock,
        stm32_open: Mock,
        can_open: Mock,
        feetech_open: Mock,
        stm32_enable: Mock,
        stm32_move: Mock,
        stm32_home: Mock,
        initialize: Mock,
        joint_command: Mock,
        torque_enable: Mock,
        position_command: Mock,
    ) -> None:
        resolver.return_value = resolved_hardware()
        runtime = create_upper_motion_runtime(hardware_config(), motion_config())

        self.assertFalse(runtime.is_open)
        stm32_open.assert_not_called()
        can_open.assert_not_called()
        feetech_open.assert_not_called()
        initialize.assert_not_called()
        stm32_enable.assert_not_called()
        stm32_home.assert_not_called()
        stm32_move.assert_not_called()
        joint_command.assert_not_called()
        torque_enable.assert_not_called()
        position_command.assert_not_called()

    @patch("bootstrap.resolve_hardware")
    def test_default_mode_is_read_only_and_profiles_are_centralized(
        self, resolver: Mock
    ) -> None:
        resolver.return_value = resolved_hardware()
        motion = motion_config()
        runtime = create_upper_motion_runtime(hardware_config(), motion)
        self.assertIs(runtime.authorization.mode, RuntimeMode.READ_ONLY)
        self.assertEqual(
            runtime.controller._arrival_configs,
            motion.arrival_configs(),
        )
        self.assertEqual(
            runtime.controller._default_motion_parameters,
            motion.default_motion_parameters(),
        )
        self.assertEqual(
            runtime.controller._linear_position_limits,
            motion.linear_position_limits(),
        )
        self.assertEqual(
            runtime.controller._linear_motion_limits,
            motion.linear_motion_limits(),
        )


class RuntimeLifecycleTests(unittest.TestCase):
    def _resources(self, events: list[str]) -> tuple[Mock, Mock, Mock]:
        def resource(name: str) -> Mock:
            item = Mock()
            item.open.side_effect = lambda: events.append(f"open:{name}")
            item.close.side_effect = lambda: events.append(f"close:{name}")
            return item

        return resource("stm32"), resource("can"), resource("feetech")

    def test_open_and_close_order_and_repeated_calls(self) -> None:
        events: list[str] = []
        stm32, can_bus, feetech = self._resources(events)
        runtime = lifecycle_runtime(stm32, can_bus, feetech)

        runtime.open()
        runtime.open()
        self.assertTrue(runtime.is_open)
        runtime.close()
        runtime.close()

        self.assertFalse(runtime.is_open)
        self.assertEqual(
            events,
            [
                "open:stm32",
                "open:can",
                "open:feetech",
                "close:feetech",
                "close:can",
                "close:stm32",
            ],
        )

    def test_stm32_open_failure_does_not_continue(self) -> None:
        stm32, can_bus, feetech = Mock(), Mock(), Mock()
        stm32.open.side_effect = RuntimeError("stm32 failed")
        runtime = lifecycle_runtime(stm32, can_bus, feetech)
        with self.assertRaisesRegex(HardwareOpenError, "STM32.*rollback") as failure:
            runtime.open()
        self.assertIsInstance(failure.exception.__cause__, RuntimeError)
        can_bus.open.assert_not_called()
        feetech.open.assert_not_called()
        stm32.close.assert_not_called()
        self.assertFalse(runtime.is_open)

    def test_can_open_failure_rolls_back_stm32(self) -> None:
        events: list[str] = []
        stm32, can_bus, feetech = self._resources(events)

        def fail_can() -> None:
            events.append("open:can")
            raise RuntimeError("can failed")

        can_bus.open.side_effect = fail_can
        runtime = lifecycle_runtime(stm32, can_bus, feetech)
        with self.assertRaisesRegex(HardwareOpenError, "CAN bus.*rollback completed"):
            runtime.open()
        self.assertEqual(events, ["open:stm32", "open:can", "close:stm32"])
        self.assertFalse(runtime.is_open)

    def test_feetech_open_failure_rolls_back_can_then_stm32(self) -> None:
        events: list[str] = []
        stm32, can_bus, feetech = self._resources(events)

        def fail_feetech() -> None:
            events.append("open:feetech")
            raise RuntimeError("feetech failed")

        feetech.open.side_effect = fail_feetech
        runtime = lifecycle_runtime(stm32, can_bus, feetech)
        with self.assertRaises(HardwareOpenError):
            runtime.open()
        self.assertEqual(
            events,
            [
                "open:stm32",
                "open:can",
                "open:feetech",
                "close:can",
                "close:stm32",
            ],
        )
        self.assertFalse(runtime.is_open)

    def test_rollback_close_error_does_not_replace_open_cause(self) -> None:
        stm32, can_bus, feetech = Mock(), Mock(), Mock()
        can_bus.open.side_effect = ValueError("original open failure")
        stm32.close.side_effect = RuntimeError("rollback close failure")
        runtime = lifecycle_runtime(stm32, can_bus, feetech)
        with self.assertRaisesRegex(HardwareOpenError, "rollback attempted.*close failure") as failure:
            runtime.open()
        self.assertIsInstance(failure.exception.__cause__, ValueError)
        self.assertFalse(runtime.is_open)

    def test_close_failure_continues_and_aggregates(self) -> None:
        events: list[str] = []
        stm32, can_bus, feetech = self._resources(events)

        def fail_close() -> None:
            events.append("close:feetech")
            raise RuntimeError("feetech close failed")

        feetech.close.side_effect = fail_close
        runtime = lifecycle_runtime(stm32, can_bus, feetech)
        runtime.open()
        with self.assertRaisesRegex(HardwareCloseError, "Feetech.*close failed"):
            runtime.close()
        self.assertEqual(events[-3:], ["close:feetech", "close:can", "close:stm32"])
        self.assertFalse(runtime.is_open)

    def test_context_closes_on_normal_and_exceptional_exit(self) -> None:
        for body_raises in (False, True):
            with self.subTest(body_raises=body_raises):
                events: list[str] = []
                resources = self._resources(events)
                runtime = lifecycle_runtime(*resources)
                if body_raises:
                    with self.assertRaisesRegex(RuntimeError, "body failure"):
                        with runtime:
                            raise RuntimeError("body failure")
                else:
                    with runtime:
                        self.assertTrue(runtime.is_open)
                self.assertEqual(events[-3:], ["close:feetech", "close:can", "close:stm32"])


class MotionAuthorizationTests(unittest.TestCase):
    def _controller(
        self,
        mode: RuntimeMode,
        *,
        allow_rotation: bool = False,
    ) -> tuple[UnifiedMotionController, Mock, Mock]:
        stm32 = Mock()
        stm32.query_axis.return_value = AxisStatus(
            "S", True, True, False, True, True, 0, 0
        )
        stm32.submit_move_absolute.return_value = object()
        rotation = Mock()
        rotation.config = END_EFFECTOR_ROTATION_CONFIG
        rotation.command_position.return_value = 1
        rotation.read_feedback.return_value = SimpleNamespace(
            position_rad=0.0,
            moving=False,
            error_raw=0,
        )
        rotation.torque_enabled.return_value = True
        shoulder = Mock()
        shoulder.config = SHOULDER_JOINT_CONFIG
        shoulder.is_enabled.return_value = True
        elbow = Mock()
        elbow.config = ELBOW_JOINT_CONFIG
        elbow.is_enabled.return_value = True
        authorization = MotionAuthorization(mode, allow_rotation)
        controller = UnifiedMotionController(
            stm32_client=stm32,
            shoulder_joint=shoulder,
            elbow_joint=elbow,
            rotation_axis=rotation,
            linear_position_limits=motion_config().linear_position_limits(),
            linear_motion_limits=motion_config().linear_motion_limits(),
            arrival_configs=motion_config().arrival_configs(),
            default_motion_parameters=motion_config().default_motion_parameters(),
            authorization=authorization,
        )
        return controller, stm32, rotation

    def test_read_only_queries_but_rejects_all_motion_entrances(self) -> None:
        controller, stm32, rotation = self._controller(RuntimeMode.READ_ONLY)
        self.assertTrue(controller.get_state(AxisName.SLIDE).connected)
        with self.assertRaises(MotionAuthorizationError):
            controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        with self.assertRaises(MotionAuthorizationError):
            controller.submit_positions(
                MultiAxisTarget((AxisTarget(AxisName.SLIDE, 1.0),))
            )
        with self.assertRaises(MotionAuthorizationError):
            controller.home_reference(AxisName.SLIDE)
        stm32.submit_move_absolute.assert_not_called()
        stm32.submit_home.assert_not_called()
        rotation.command_position.assert_not_called()

    def test_motion_mode_allows_ordinary_axis_without_automatic_command(self) -> None:
        controller, stm32, _rotation = self._controller(RuntimeMode.MOTION)
        stm32.submit_move_absolute.assert_not_called()
        controller.submit_absolute(AxisTarget(AxisName.SLIDE, 1.0))
        stm32.submit_move_absolute.assert_called_once()

    def test_rotation_requires_extra_authorization_for_single_and_multi_axis(self) -> None:
        controller, stm32, rotation = self._controller(RuntimeMode.MOTION)
        with self.assertRaisesRegex(
            RotationMotionAuthorizationError,
            "no verified independent stop",
        ):
            controller.submit_absolute(AxisTarget(AxisName.ROTATION, 1.0))
        with self.assertRaises(RotationMotionAuthorizationError):
            controller.submit_positions(
                MultiAxisTarget(
                    (
                        AxisTarget(AxisName.SLIDE, 1.0),
                        AxisTarget(AxisName.ROTATION, 1.0),
                    )
                )
            )
        stm32.submit_move_absolute.assert_not_called()
        rotation.command_position.assert_not_called()

    def test_explicit_rotation_authorization_enters_existing_submission(self) -> None:
        controller, _stm32, rotation = self._controller(
            RuntimeMode.MOTION,
            allow_rotation=True,
        )
        controller.submit_absolute(AxisTarget(AxisName.ROTATION, 1.0))
        rotation.command_position.assert_called_once()

    def test_stop_is_safety_action_but_rotation_stop_remains_unsupported(self) -> None:
        controller, stm32, rotation = self._controller(RuntimeMode.READ_ONLY)
        slide_result = controller.stop(AxisName.SLIDE)
        self.assertEqual(slide_result.status.value, "aborted")
        stm32.stop.assert_called_once_with("slide")
        rotation_result = controller.stop(AxisName.ROTATION)
        self.assertEqual(rotation_result.error_code, MotionErrorCode.UNSUPPORTED_COMMAND)
        rotation.disable_torque.assert_not_called()


class ReadOnlySmokeTests(unittest.TestCase):
    def test_smoke_queries_all_hardware_and_closes_without_control_calls(self) -> None:
        runtime = MagicMock()
        runtime.__enter__.return_value = runtime
        runtime.__exit__.return_value = None
        runtime.stm32_client.version.return_value = SimpleNamespace(
            protocol_version="0.2",
            firmware_version="test",
        )
        runtime.shoulder_joint.initialize.return_value = SimpleNamespace(
            position_rad=0.1
        )
        runtime.elbow_joint.initialize.return_value = SimpleNamespace(position_rad=-0.2)
        runtime.controller.get_axis_states.return_value = tuple(
            SimpleNamespace(
                axis=axis,
                connected=True,
                enabled=None,
                busy=False,
                homed=None,
                position_valid=True,
                current_position=0.0,
                position_unit="mm" if axis in (AxisName.SLIDE, AxisName.Z) else "deg",
                faulted=False,
                fault_code=None,
                fault_message=None,
            )
            for axis in AxisName
        )
        output: list[str] = []

        run_read_only_smoke(runtime, emit=output.append)

        runtime.__enter__.assert_called_once()
        runtime.__exit__.assert_called_once()
        runtime.stm32_client.version.assert_called_once()
        runtime.shoulder_joint.initialize.assert_called_once()
        runtime.elbow_joint.initialize.assert_called_once()
        runtime.controller.get_axis_states.assert_called_once()
        self.assertIn("stm32", "\n".join(output))
        self.assertIn("axis=rotation", "\n".join(output))
        for forbidden in (
            runtime.controller.submit_absolute,
            runtime.controller.submit_positions,
            runtime.controller.home_reference,
            runtime.rotation_axis.enable_torque,
        ):
            forbidden.assert_not_called()

    @patch("scripts.manual_motion.run_inspect")
    @patch("scripts.manual_motion.create_configured_runtime")
    def test_smoke_main_defaults_to_read_only(
        self,
        create_runtime: Mock,
        run_smoke: Mock,
    ) -> None:
        create_runtime.return_value = Mock()
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(smoke_main(["inspect"]), 0)
        create_runtime.assert_called_once_with(
            RuntimeMode.READ_ONLY,
            allow_unverified_rotation_motion=False,
        )
        run_smoke.assert_called_once_with(create_runtime.return_value)

    def test_smoke_closes_when_a_read_fails(self) -> None:
        runtime = MagicMock()
        runtime.__enter__.return_value = runtime
        runtime.__exit__.return_value = None
        runtime.stm32_client.version.side_effect = RuntimeError("read failed")
        with self.assertRaisesRegex(RuntimeError, "read failed"):
            run_read_only_smoke(runtime)
        runtime.__exit__.assert_called_once()


class BootstrapImportTests(unittest.TestCase):
    @patch("drivers.feetech_protocol.FeetechBus.open")
    @patch("drivers.can_bus.CanMotorBus.open")
    @patch("drivers.stm32_motion.STM32SerialTransport.open")
    @patch("drivers.device_discovery.resolve_hardware")
    def test_import_has_no_discovery_or_open_side_effects(
        self,
        resolve: Mock,
        stm32_open: Mock,
        can_open: Mock,
        feetech_open: Mock,
    ) -> None:
        path = Path(__file__).resolve().parents[1] / "bootstrap.py"
        spec = importlib.util.spec_from_file_location("bootstrap_side_effect_test", path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        resolve.assert_not_called()
        stm32_open.assert_not_called()
        can_open.assert_not_called()
        feetech_open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
