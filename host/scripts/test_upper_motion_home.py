#!/usr/bin/env python3
"""Deprecated compatibility entry point for Slide/Z reference home."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from scripts.debug_motion.home_linear_axis import (  # noqa: E402
    build_parser,
    main as _new_main,
    run_home_test,
)


def main(argv: Sequence[str] | None = None) -> int:
    print(
        "DEPRECATED: Use scripts/debug_motion/home_linear_axis.py instead.",
        file=sys.stderr,
    )
    return _new_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
