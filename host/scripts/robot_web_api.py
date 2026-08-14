#!/usr/bin/env python3
"""启动 MushroomRobotService 的本地 FastAPI 适配器。"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import sys
from pathlib import Path


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

import uvicorn  # noqa: E402

from application.robot_service import MushroomRobotService  # noqa: E402
from application.web_api import create_robot_web_app  # noqa: E402
from scripts.robot_service import (  # noqa: E402
    build_parser as build_service_parser,
    create_service,
    validate_args as validate_service_args,
)


def build_parser() -> argparse.ArgumentParser:
    parser = build_service_parser()
    parser.description = "Mushroom Robot Service Web API"
    parser.add_argument("--host", default="172.20.10.3")
    parser.add_argument("--port", type=_port, default=8000)
    parser.add_argument(
        "--cors-origin",
        action="append",
        dest="cors_origins",
        help="allowed browser origin; repeat to allow multiple origins",
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    service_factory: Callable[..., MushroomRobotService] = create_service,
    uvicorn_runner: Callable[..., object] = uvicorn.run,
) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_service_args(parser, args)

    service = None
    app = None
    exit_code = 0
    try:
        service = service_factory(args)
        app = create_robot_web_app(
            service,
            allowed_origins=args.cors_origins,
        )
        uvicorn_runner(app, host=args.host, port=args.port)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"robot web api error: {exc}", file=sys.stderr)
        exit_code = 2
    finally:
        if app is not None:
            try:
                app.state.shutdown_service_once()
            except Exception as exc:
                print(f"robot web api shutdown error: {exc}", file=sys.stderr)
                exit_code = 2
        elif service is not None:
            try:
                service.shutdown()
            except Exception as exc:
                print(f"robot web api shutdown error: {exc}", file=sys.stderr)
                exit_code = 2
    return exit_code


def _port(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return port


if __name__ == "__main__":
    raise SystemExit(main())
