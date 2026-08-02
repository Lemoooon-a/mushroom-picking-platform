#!/usr/bin/env python3
"""用平面二连杆逆运动学预览或显式测试肩肘双关节。"""

from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path
import sys

import can


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from config.joints import (  # noqa: E402
    ELBOW_JOINT_CONFIG,
    SHOULDER_JOINT_CONFIG,
)
from drivers.can_bus import CanMotorBus  # noqa: E402
from drivers.mg4010_driver import MG4010Driver  # noqa: E402
from drivers.mg4010_protocol import MotorError  # noqa: E402
from kinematics import KinematicsError, Planar2RKinematics  # noqa: E402
from robot import (  # noqa: E402
    CanRotaryJoint,
    JointError,
    Planar2RArmController,
    PlanarArmError,
    PlanarArmTarget,
    select_joint_target,
)


LOGGER = logging.getLogger("test_planar_2r_motion")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Solve a calibrated planar 2R XY target and optionally submit "
            "back-to-back shoulder/elbow 0xA4 commands. Without --enable-motion "
            "this tool is fully offline and does not open CAN."
        )
    )
    parser.add_argument("--link1-length", type=_positive_float, required=True)
    parser.add_argument("--link2-length", type=_positive_float, required=True)
    parser.add_argument("--x", type=_finite_float, required=True)
    parser.add_argument("--y", type=_finite_float, required=True)
    parser.add_argument(
        "--elbow-branch",
        choices=("positive", "negative"),
        default="positive",
    )
    parser.add_argument("--velocity-rad-s", type=_positive_float, required=True)
    parser.add_argument("--interface", choices=("gs_usb", "socketcan"))
    parser.add_argument("--channel")
    parser.add_argument("--bitrate", type=_positive_int)
    parser.add_argument("--allow-same-id-response", action="store_true")
    parser.add_argument("--raw", action="store_true")
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="open CAN and explicitly permit both calibrated A4 submissions",
    )
    return parser


def _finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def _positive_float(value: str) -> float:
    parsed = _finite_float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def _motion_frame_printer(show_all_frames: bool):
    """实机运动时始终显示最终 A4/81 发送帧。"""

    def print_selected(direction: str, message: can.Message) -> None:
        is_control_tx = (
            direction == "TX"
            and bool(message.data)
            and message.data[0] in (0xA4, 0x81)
        )
        if is_control_tx:
            data = " ".join(f"{byte:02X}" for byte in message.data)
            print(
                f"FINAL-CONTROL-TX 0x{message.arbitration_id:03X} "
                f"[{message.dlc}] {data}"
            )
        elif show_all_frames:
            data = " ".join(f"{byte:02X}" for byte in message.data)
            print(f"{direction} 0x{message.arbitration_id:03X} [{message.dlc}] {data}")

    return print_selected


def print_target(
    target: PlanarArmTarget,
    kinematics: Planar2RKinematics,
    velocity_rad_s: float,
) -> None:
    recovered = kinematics.forward(
        target.angles.shoulder_rad,
        target.angles.elbow_rad,
    )
    print(f"target XY                  : ({target.point.x:.6f}, {target.point.y:.6f})")
    print(
        "selected joint radians     : "
        f"shoulder={target.angles.shoulder_rad:+.9f}, "
        f"elbow={target.angles.elbow_rad:+.9f}"
    )
    print(
        "selected joint degrees     : "
        f"shoulder={math.degrees(target.angles.shoulder_rad):+.6f}, "
        f"elbow={math.degrees(target.angles.elbow_rad):+.6f}"
    )
    print(f"requested joint speed      : {velocity_rad_s:.9f} rad/s")
    print(f"FK verification XY         : ({recovered.x:.6f}, {recovered.y:.6f})")


def _live_run(
    args: argparse.Namespace,
    kinematics: Planar2RKinematics,
    target: PlanarArmTarget,
) -> int:
    print("DUAL-JOINT MOTION ENABLED")
    with CanMotorBus(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
        allow_same_id_response=args.allow_same_id_response,
        raw_frame_callback=_motion_frame_printer(args.raw),
    ) as bus:
        shoulder = CanRotaryJoint(
            MG4010Driver(bus, SHOULDER_JOINT_CONFIG.motor_id),
            SHOULDER_JOINT_CONFIG,
        )
        elbow = CanRotaryJoint(
            MG4010Driver(bus, ELBOW_JOINT_CONFIG.motor_id),
            ELBOW_JOINT_CONFIG,
        )
        shoulder_initial = shoulder.initialize()
        elbow_initial = elbow.initialize()
        print(
            "current joint degrees      : "
            f"shoulder={math.degrees(shoulder_initial.position_rad):+.6f}, "
            f"elbow={math.degrees(elbow_initial.position_rad):+.6f}"
        )
        controller = Planar2RArmController(kinematics, shoulder, elbow)
        controller.command_target(
            target,
            shoulder_velocity_rad_s=args.velocity_rad_s,
            elbow_velocity_rad_s=args.velocity_rad_s,
        )
        print(
            "Both position commands were accepted; mechanical arrival and "
            "simultaneous arrival are not implied"
        )
    return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_argument_parser().parse_args(argv)
    try:
        kinematics = Planar2RKinematics(
            args.link1_length,
            args.link2_length,
        )
        target = select_joint_target(
            kinematics,
            SHOULDER_JOINT_CONFIG,
            ELBOW_JOINT_CONFIG,
            args.x,
            args.y,
            elbow_branch=args.elbow_branch,
        )
        print_target(target, kinematics, args.velocity_rad_s)
        if not args.enable_motion:
            print("OFFLINE PREVIEW - CAN WAS NOT OPENED; NO MOTOR COMMAND WAS SENT")
            return 0
        return _live_run(args, kinematics, target)
    except (
        JointError,
        KinematicsError,
        MotorError,
        PlanarArmError,
        can.CanError,
        OSError,
        ValueError,
    ) as exc:
        LOGGER.error("planar 2R joint test failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
