#!/usr/bin/env python3
"""Deprecated compatibility entry point for five-axis point-to-point debug."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import sys


HOST_ROOT = Path(__file__).resolve().parents[1]
if str(HOST_ROOT) not in sys.path:
    sys.path.insert(0, str(HOST_ROOT))

from scripts.debug_motion.debug_multi_axis_motion import (  # noqa: E402
    legacy_main as _new_legacy_main,
    run_five_axis_test,
    run_multi_axis_test,
)


def main(argv: Sequence[str] | None = None) -> int:
    print(
        "DEPRECATED: Use scripts/debug_motion/debug_multi_axis_motion.py instead.",
        file=sys.stderr,
    )
    return _new_legacy_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
