#!/usr/bin/env python3
"""统一 Robot Service 命令入口；默认不允许真实运动。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import json
from pathlib import Path
import shlex
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = HOST_ROOT.parent
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from application.controller import MushroomRobotController  # noqa: E402
from application.demo_backend import (  # noqa: E402
    DemoFlowApplicationBackend,
)
from application.offline_backend import create_offline_planning_controller  # noqa: E402
from application.execution_record import JsonLinesExecutionRecorder  # noqa: E402
from application.motion_target import BaseToolTarget  # noqa: E402
from application.pick_planner import PickPlanner  # noqa: E402
from application.pick_workflow import VisionPickWorkflow  # noqa: E402
from application.robot_service import (  # noqa: E402
    MushroomRobotService,
    ResolvedCameraPoint,
)
from application.runtime_state import RobotServiceMode  # noqa: E402
from application.tray_workspace import TrayWorkspace  # noqa: E402
from calibration.hand_eye import hand_eye_from_frame_document  # noqa: E402
from config.frame_transforms import load_frame_transforms_document  # noqa: E402
from config.project.grasp_strategy import load_validated_grasp_profile  # noqa: E402
from config.project.vision_runtime import (  # noqa: E402
    DEFAULT_VISION_RUNTIME_CONFIG, VisionRuntimeConfig, load_vision_runtime_config,
)
from config.tray_workspace import load_tray_workspace_config  # noqa: E402
from scripts.run_motion_demo import create_demo_flow  # noqa: E402
from motion.unified_protocol import (  # noqa: E402
    AxisDescriptor, AxisKind, AxisName, AxisState, MotionCommandResult,
)
from vision.gateway import FakeVisionGateway, JsonSocketVisionGateway  # noqa: E402
from vision.observation import Vector3  # noqa: E402
from vision.protocol import NoTarget, TargetDetection  # noqa: E402
from vision.target_resolver import VisionTargetResolver  # noqa: E402


DEFAULT_FRAME_CONFIG = HOST_ROOT / "config" / "local" / "frame_transforms.json"
DEFAULT_TRAY_CONFIG = HOST_ROOT / "config" / "local" / "tray_workspace.json"
DEFAULT_VISION_CONFIG = HOST_ROOT / "config" / "local" / "vision_runtime.json"
DEFAULT_GRASP_CONFIG = HOST_ROOT / "config" / "local" / "grasp_profile.json"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Top-level mushroom robot service")
    parser.add_argument("--mode", choices=[item.value for item in RobotServiceMode], default=RobotServiceMode.READ_ONLY.value)
    parser.add_argument("--confirm-motion", action="store_true", help="required with --mode execute")
    parser.add_argument("--confirm-rotation-no-stop", action="store_true", help="accept existing Rotation stop limitation in execute mode")
    parser.add_argument("--frame-config", type=Path, default=DEFAULT_FRAME_CONFIG)
    parser.add_argument("--tray-workspace-config", type=Path, default=DEFAULT_TRAY_CONFIG)
    parser.add_argument("--vision-runtime-config", type=Path, default=DEFAULT_VISION_CONFIG)
    parser.add_argument("--grasp-profile-config", type=Path, default=DEFAULT_GRASP_CONFIG)
    parser.add_argument("--vision-gateway", choices=("fake", "socket"), default="fake")
    parser.add_argument("--fake-position", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--fake-confidence", type=float, default=0.95)
    parser.add_argument("--record-jsonl", type=Path)
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    execute = args.mode == RobotServiceMode.EXECUTE.value
    if execute != bool(args.confirm_motion):
        parser.error("execute mode requires --confirm-motion")
    if execute != bool(args.confirm_rotation_no_stop):
        parser.error("execute mode requires --confirm-rotation-no-stop")
    if args.mode == RobotServiceMode.READ_ONLY.value and args.fake_position is not None:
        parser.error("--fake-position is only valid in dry-run or execute mode")
    if args.vision_gateway == "socket" and args.fake_position is not None:
        parser.error("--fake-position requires --vision-gateway fake")


def create_service(args: argparse.Namespace, *, emit: Callable[[str], None] = print) -> MushroomRobotService:
    mode = RobotServiceMode(args.mode)
    execute = mode is RobotServiceMode.EXECUTE
    vision_config = (
        load_vision_runtime_config(args.vision_runtime_config)
        if args.vision_runtime_config.exists()
        else DEFAULT_VISION_RUNTIME_CONFIG
    )
    if execute:
        runtime, flow = create_demo_flow(execute=True, frame_config=args.frame_config, emit=emit)
        backend = DemoFlowApplicationBackend(runtime=runtime, flow=flow)
        document = load_frame_transforms_document(args.frame_config)
        resolver = VisionTargetResolver(
            pose_provider=flow.solver,
            hand_eye_calibration=hand_eye_from_frame_document(document, source=str(args.frame_config)),
            camera_frame_id=vision_config.camera_frame,
        )
        controller = MushroomRobotController(
            base_backend=backend,
            tray_workspace=TrayWorkspace(load_tray_workspace_config(args.tray_workspace_config)),
            target_resolver=resolver,
        )
    else:
        runtime = None
        controller, backend = create_offline_planning_controller(
            frame_config=args.frame_config,
            tray_workspace_config=args.tray_workspace_config,
            camera_frame=vision_config.camera_frame,
        )
    gateway, description = _gateway(args, vision_config)
    planner = PickPlanner(controller)
    workflow = VisionPickWorkflow(
        controller=controller,
        gateway=gateway,
        planner=planner,
        capture_state_reader=backend.capture_state,
        joints_holding=backend.joints_holding,
        camera_frame=vision_config.camera_frame,
        timeout_s=vision_config.timeout_s,
    )
    profile = None
    if args.grasp_profile_config.exists():
        try:
            profile = load_validated_grasp_profile(args.grasp_profile_config)
        except ValueError as exc:
            emit(f"Grasp profile unavailable: {exc}")
    recorder = None
    if args.record_jsonl is not None:
        recorder = JsonLinesExecutionRecorder(args.record_jsonl, repository_root=REPOSITORY_ROOT)
    return MushroomRobotService(
        axis_motion=runtime.controller if runtime is not None else backend,
        controller=controller,
        workflow=workflow,
        mode=mode,
        grasp_profile=profile,
        recorder=recorder,
        activate_controller_on_startup=mode is not RobotServiceMode.READ_ONLY,
        vision_gateway_description=description,
    )


def _gateway(args: argparse.Namespace, config: VisionRuntimeConfig):
    if args.vision_gateway == "socket":
        if not config.validated or config.host is None or config.port is None:
            raise ValueError("real socket vision config is missing or not validated")
        return JsonSocketVisionGateway(config.host, config.port, maximum_message_bytes=config.maximum_message_bytes), "real socket configurable"

    def respond(request):
        if args.fake_position is None:
            return NoTarget(request.request_id, "no_synthetic_target_configured")
        return TargetDetection(
            request_id=request.request_id,
            frame_id=request.camera_frame,
            timestamp=request.timestamp,
            target_id="synthetic-target",
            confidence=args.fake_confidence,
            position_mm=Vector3(*args.fake_position),
            orientation=None,
        )

    return FakeVisionGateway(responder=respond), "fake available"


class RobotServiceShell:
    def __init__(self, service: MushroomRobotService, *, emit: Callable[[str], None] = print) -> None:
        self.service = service
        self.emit = emit
        self.latest_observation = None

    def run_command(self, line: str) -> bool:
        parts = shlex.split(line)
        if not parts:
            return True
        command = parts[0].lower()
        if command in ("quit", "exit"):
            self.service.shutdown()
            return False
        if command == "help":
            self.emit("status | capabilities | workspace | startup | return | stop")
            self.emit("move x y z [yaw] | plan x y z [yaw] | observe | plan-observation | pick")
            self.emit("resolve-camera-point x_mm y_mm z_mm")
            self.emit("suction grip|release|idle | joints enable|disable | quit")
            self.emit("axes | axis state <axis> | axis states [axis ...]")
            self.emit("axis move-abs <axis> <position> [--velocity V] [--acceleration A] [--timeout T]")
            self.emit("axis move-rel <axis> <delta> [--velocity V] [--acceleration A] [--timeout T]")
            self.emit("Single-axis commands are raw/manual maintenance operations.")
            self.emit(
                "They enforce only the selected axis state and soft limits; no Base-frame "
                "workspace, IK, offset-workspace, side-switch, collision, or path checks apply."
            )
            return True
        if command == "status":
            self.emit(_json(self.service.status()))
            return True
        if command == "capabilities":
            for line_item in format_capabilities(self.service):
                self.emit(line_item)
            return True
        if command == "workspace":
            self.emit(_json(self.service.tray_workspace.config))
            return True
        if command == "axes" and len(parts) == 1:
            for descriptor in self.service.list_axes():
                self.emit(format_axis_descriptor(descriptor))
            return True
        if command == "axis":
            return self._run_axis_command(parts)
        if command == "startup":
            self.service.startup()
            self.emit("Robot Service READY")
            return True
        if command == "return":
            self.service.return_to_startup()
            return True
        if command == "stop":
            self.service.stop()
            return True
        if command in ("move", "plan"):
            target = _target(parts)
            result = self.service.move_base_target(target) if command == "move" else self.service.plan_base_target(target)
            self.emit(_json(result))
            return True
        if command == "observe":
            self.latest_observation = self.service.request_observation()
            self.emit(_json(self.latest_observation))
            return True
        if command == "resolve-camera-point":
            x_mm, y_mm, z_mm = _camera_point(parts)
            for line_item in format_resolved_camera_point(
                self.service.resolve_camera_point(x_mm, y_mm, z_mm)
            ):
                self.emit(line_item)
            return True
        if command == "plan-observation":
            if self.latest_observation is None:
                raise ValueError("observe must succeed before plan-observation")
            self.emit(_json(self.service.plan_observation(self.latest_observation)))
            return True
        if command == "pick":
            self.emit(_json(self.service.pick()))
            return True
        if command == "suction" and len(parts) == 2:
            self.service.suction(parts[1])
            return True
        if command == "joints" and len(parts) == 2:
            if parts[1] == "enable": self.service.enable_joints()
            elif parts[1] == "disable": self.service.disable_joints()
            else: raise ValueError("joints action must be enable or disable")
            return True
        raise ValueError("unknown command; type help")

    def _run_axis_command(self, parts: list[str]) -> bool:
        if len(parts) < 2:
            raise ValueError("usage: axis state|states|move-abs|move-rel ...")
        action = parts[1]
        if action == "state" and len(parts) == 3:
            self.emit(format_axis_state(self.service.get_axis_state(_axis(parts[2]))))
            return True
        if action == "states":
            axes = None if len(parts) == 2 else tuple(_axis(item) for item in parts[2:])
            for state in self.service.get_axis_states(axes):
                self.emit(format_axis_state(state))
            return True
        if action in ("move-abs", "move-rel"):
            axis, value, options = _axis_move_arguments(parts)
            if action == "move-abs":
                result = self.service.move_axis_absolute(axis, value, **options)
                self.emit(format_axis_motion_result("absolute", result))
            else:
                result = self.service.move_axis_relative(axis, value, **options)
                self.emit(
                    format_axis_motion_result(
                        "relative", result, requested_delta=value
                    )
                )
            return True
        raise ValueError("usage: axis state|states|move-abs|move-rel ...")

    def command_loop(self) -> None:
        self.emit('Type "help" for commands.')
        while True:
            try:
                line = input("robot-service> ")
                if not self.run_command(line):
                    return
            except EOFError:
                self.run_command("quit")
                return
            except KeyboardInterrupt:
                self.emit("Ctrl+C received; requesting stop and shutdown")
                try: self.service.stop()
                finally: self.service.shutdown()
                return
            except Exception as exc:
                self.emit(f"ERROR: {exc}")


def format_capabilities(service: MushroomRobotService) -> tuple[str, ...]:
    capabilities = service.capabilities
    available = lambda value: "available" if value else "unavailable"
    return (
        f"Base-frame motion: {available(capabilities.base_frame_motion)}",
        f"Tray workspace gate: {available(capabilities.tray_workspace_gate)}",
        f"Offset planning: {available(capabilities.offset_planning)}",
        f"Robot motion envelope: {available(capabilities.robot_motion_envelope)}",
        f"Joint holding: {available(capabilities.joint_holding)}",
        f"Suction command: {available(capabilities.suction_command)}",
        f"Axis listing: {available(capabilities.axis_listing)}",
        f"Axis state query: {available(capabilities.axis_state_query)}",
        f"Axis absolute motion: {available(capabilities.axis_absolute_motion)}",
        f"Axis relative motion: {available(capabilities.axis_relative_motion)}",
        f"Vision gateway: {capabilities.vision_gateway}",
        f"Vision target observation: {available(capabilities.vision_target_observation)}",
        f"Hand-eye calibration: {capabilities.hand_eye_calibration.value}",
        f"Vision target resolution: {available(capabilities.vision_target_resolution)}",
        f"Pick planning: {available(capabilities.pick_planning)}",
        f"Pick execution: {available(capabilities.pick_execution)}",
        "Physical pick verification: unavailable",
    )


def format_axis_descriptor(descriptor: AxisDescriptor) -> str:
    if not isinstance(descriptor, AxisDescriptor):
        raise TypeError("descriptor must be an AxisDescriptor")
    return _json(
        {
            "name": descriptor.name.value,
            "unit": descriptor.position_unit,
            "minimum": descriptor.minimum_position,
            "maximum": descriptor.maximum_position,
            "supports_homing": descriptor.capabilities.reference_home,
            "requires_holding": descriptor.kind is AxisKind.ROTARY,
        }
    )


def format_axis_state(state: AxisState) -> str:
    if not isinstance(state, AxisState):
        raise TypeError("state must be an AxisState")
    return _json(
        {
            "axis": state.axis.value,
            "position": state.current_position,
            "unit": state.position_unit,
            "position_valid": state.position_valid,
            "homed": state.homed,
            "moving": state.busy,
            "enabled_holding": state.enabled,
            "fault": state.faulted,
            "fault_code": state.fault_code,
            "fault_message": state.fault_message,
        }
    )


def format_axis_motion_result(
    command_kind: str,
    result: MotionCommandResult,
    *,
    requested_delta: float | None = None,
) -> str:
    if not isinstance(result, MotionCommandResult):
        raise TypeError("result must be a MotionCommandResult")
    no_op = "no motion submitted" in result.message
    return _json(
        {
            "command_type": command_kind,
            "axis": result.axis.value,
            "start_position": (
                result.target_position - requested_delta
                if requested_delta is not None
                else None
            ),
            "requested_delta": requested_delta,
            "absolute_target": result.target_position,
            "submitted": result.accepted and not no_op,
            "no_op": no_op,
            "terminal_status": result.status.value,
            "message": result.message,
        }
    )


def format_resolved_camera_point(
    result: ResolvedCameraPoint,
) -> tuple[str, ...]:
    if not isinstance(result, ResolvedCameraPoint):
        raise TypeError("result must be a ResolvedCameraPoint")
    camera_x, camera_y, camera_z = result.camera_point_mm
    base_x, base_y, base_z = result.base_point_mm
    return (
        "Camera point:",
        f"  frame: {result.frame_id}",
        f"  x: {camera_x:.3f} mm",
        f"  y: {camera_y:.3f} mm",
        f"  z: {camera_z:.3f} mm",
        "",
        "Base point:",
        "  frame: base",
        f"  x: {base_x:.3f} mm",
        f"  y: {base_y:.3f} mm",
        f"  z: {base_z:.3f} mm",
        "",
        "Transform status:",
        f"  tool_T_camera source: {result.tool_camera_source}",
        "  tool_T_camera validated: "
        f"{str(result.tool_camera_validated).lower()}",
        f"  result: {result.transform_status.value.upper()}",
    )


def _camera_point(parts: list[str]) -> tuple[float, float, float]:
    if len(parts) != 4:
        raise ValueError("usage: resolve-camera-point x_mm y_mm z_mm")
    return float(parts[1]), float(parts[2]), float(parts[3])


def _axis(value: str) -> AxisName:
    try:
        return AxisName(value.lower())
    except ValueError as exc:
        raise ValueError(f"unknown axis {value!r}") from exc


def _axis_move_arguments(
    parts: list[str],
) -> tuple[AxisName, float, dict[str, float | None]]:
    if len(parts) < 4:
        raise ValueError(
            f"usage: axis {parts[1]} <axis> <value> "
            "[--velocity V] [--acceleration A] [--timeout T]"
        )
    axis = _axis(parts[2])
    value = float(parts[3])
    options: dict[str, float | None] = {
        "velocity": None,
        "acceleration": None,
        "timeout_s": None,
    }
    names = {
        "--velocity": "velocity",
        "--acceleration": "acceleration",
        "--timeout": "timeout_s",
    }
    index = 4
    while index < len(parts):
        option = parts[index]
        if option not in names or index + 1 >= len(parts):
            raise ValueError(f"invalid axis motion option {option!r}")
        options[names[option]] = float(parts[index + 1])
        index += 2
    return axis, value, options


def _target(parts: list[str]) -> BaseToolTarget:
    if len(parts) not in (4, 5):
        raise ValueError(f"usage: {parts[0]} x y z [yaw]")
    values = [float(item) for item in parts[1:]]
    return BaseToolTarget(values[0], values[1], values[2], values[3] if len(values) == 4 else None)


def _json(value: object) -> str:
    from application.execution_record import _json_value
    return json.dumps(value, ensure_ascii=False, default=_json_value, allow_nan=False)


def main(argv: Sequence[str] | None = None, *, service_factory=create_service) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)
    try:
        service = service_factory(args)
        RobotServiceShell(service).command_loop()
        return 0
    except Exception as exc:
        print(f"robot service configuration error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
