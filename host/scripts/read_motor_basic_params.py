#!/usr/bin/env python3
"""只读检查一台瓴控 MG4010E-i36 的位置、状态和故障。"""

from __future__ import annotations

import argparse
from datetime import datetime
import logging
import math
from pathlib import Path
import sys
import time

import can


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from drivers.can_bus import (  # noqa: E402
    DEFAULT_BITRATE,
    DEFAULT_RETRIES,
    DEFAULT_TIMEOUT_SECONDS,
    CanMotorBus,
)
from drivers.mg4010_driver import MG4010Driver  # noqa: E402
from drivers.mg4010_protocol import MotorError  # noqa: E402


LOGGER = logging.getLogger("read_motor_basic_params")
DEFAULT_GEAR_RATIO = 36.0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read MG4010E-i36 0x94/0x92 positions and 0x9A/0x9C state. "
            "This tool never sends motion, enable, clear, or write commands."
        )
    )
    parser.add_argument("--motor-id", type=int, required=True, choices=range(1, 33))
    parser.add_argument("--interface", choices=("gs_usb", "socketcan"))
    parser.add_argument("--channel")
    parser.add_argument("--bitrate", type=_positive_int)
    parser.add_argument("--timeout", type=_positive_float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--retries", type=_non_negative_int, default=DEFAULT_RETRIES)
    parser.add_argument(
        "--gear-ratio",
        "--reduction-ratio",
        dest="gear_ratio",
        type=_positive_float,
        default=DEFAULT_GEAR_RATIO,
    )
    parser.add_argument(
        "--allow-same-id-response",
        action="store_true",
        help="explicitly accept observed firmware replies on 0x140+ID",
    )
    parser.add_argument("--watch-angles", action="store_true")
    parser.add_argument("--watch-interval", type=_positive_float, default=0.5)
    parser.add_argument("--watch-count", type=_non_negative_int, default=0)
    parser.add_argument("--raw", action="store_true")
    return parser


def _positive_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def _non_negative_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return parsed


def _raw_printer(direction: str, message: can.Message) -> None:
    payload = " ".join(f"{byte:02X}" for byte in message.data)
    print(f"{direction} 0x{message.arbitration_id:03X} [{message.dlc}] {payload}")


def watch_angles(
    driver: MG4010Driver,
    *,
    gear_ratio: float,
    interval: float,
    count: int,
) -> int:
    """持续只读 0x94/0x92，显示电机侧与输出轴角度。"""

    print("Time MC_deg MM_deg OA_deg dMM_deg State")
    baseline_multi: float | None = None
    sample_number = 0
    try:
        while count == 0 or sample_number < count:
            sample_number += 1
            single = driver.read_single_turn_position()
            multi = driver.read_multi_turn_position_deg()
            output_abs = (single.motor_cycle_deg / gear_ratio) % 360.0
            if baseline_multi is None:
                baseline_multi = multi
                state = "BASE"
            else:
                state = "OK"
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(
                f"{timestamp} MC={single.motor_cycle_deg:.2f} MM={multi:.2f} "
                f"OA={output_abs:.4f} dMM={multi - baseline_multi:+.2f} ST={state}",
                flush=True,
            )
            if count == 0 or sample_number < count:
                time.sleep(interval)
    except KeyboardInterrupt:
        print("\nAngle monitor stopped by user.")
        return 130
    return 0


def print_report(driver: MG4010Driver, gear_ratio: float) -> None:
    """读取并打印首版正式驱动支持的全部只读状态。"""

    single = driver.read_single_turn_position()
    multi = driver.read_multi_turn_position_deg()
    status = driver.read_status()
    fault = driver.read_fault()
    output_abs = (single.motor_cycle_deg / gear_ratio) % 360.0

    print("MG4010E-i36 read-only report")
    print(f"Motor ID                 : {driver.motor_id}")
    print(f"CAN request ID           : 0x{driver.request_id:03X}")
    print(f"Protocol response ID     : 0x{driver.response_id:03X}")
    print(f"Gear ratio               : {gear_ratio:g}:1")
    print(f"circle_angle_raw         : {single.circle_angle_raw}")
    print(f"motor_cycle_deg (0x94)   : {single.motor_cycle_deg:.2f}")
    print(f"output_abs_deg           : {output_abs:.6f}")
    print(f"motor_multi_turn_deg     : {multi:.2f}")
    print(f"motor_speed_deg_s        : {status.motor_speed_deg_s}")
    print(f"temperature_c            : {status.temperature_c}")
    print(f"torque_current_raw       : {status.torque_current_raw}")
    print(f"torque_current_a         : {status.torque_current_a:.4f}")
    print(f"encoder_raw              : {status.encoder_raw}")
    print(f"bus_voltage_v            : {fault.bus_voltage_v:.2f}")
    print(f"bus_current_a            : {fault.bus_current_a:.2f}")
    print(f"motor_state              : 0x{fault.motor_state:02X}")
    print(f"error_state              : 0x{fault.error_state:02X}")
    print("0x92 note                : current power-cycle coordinate only")


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = build_argument_parser().parse_args(argv)
    try:
        with CanMotorBus(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
            timeout=args.timeout,
            retries=args.retries,
            allow_same_id_response=args.allow_same_id_response,
            raw_frame_callback=_raw_printer if args.raw else None,
        ) as bus:
            driver = MG4010Driver(bus, args.motor_id)
            if args.watch_angles:
                return watch_angles(
                    driver,
                    gear_ratio=args.gear_ratio,
                    interval=args.watch_interval,
                    count=args.watch_count,
                )
            print_report(driver, args.gear_ratio)
            return 0
    except (MotorError, can.CanError, OSError, ValueError) as exc:
        LOGGER.error("read-only motor query failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
