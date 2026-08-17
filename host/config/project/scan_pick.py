"""固定八区域扫描抓取配置加载器。"""

from __future__ import annotations

import json
from pathlib import Path

from application.motion_target import BaseToolTarget
from application.scan_pick import ScanPickProfile


class ScanPickConfigError(ValueError):
    pass


_SCHEMA_VERSION = 4
_REQUIRED_FIELDS = frozenset(
    {
        "schema_version",
        "validated",
        "scan_x_positions_mm",
        "scan_y_positions_mm",
        "scan_yaw_deg",
        "place_pose",
        "oversized_place_pose",
        "max_picks_per_scan_pose",
    }
)
_OPTIONAL_FIELDS = frozenset({"scan_settle_time_s"})
_PLACE_FIELDS = frozenset({"x_mm", "y_mm", "z_mm", "yaw_deg"})


def load_validated_scan_pick_profile(path: Path) -> ScanPickProfile:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScanPickConfigError(f"cannot load scan-pick profile {path}: {exc}") from exc
    return parse_validated_scan_pick_profile(root)


def parse_validated_scan_pick_profile(root: object) -> ScanPickProfile:
    """校验统一 Runtime 配置中的 ``scan_pick`` 区块。"""

    if not isinstance(root, dict) or root.get("schema_version") != _SCHEMA_VERSION:
        raise ScanPickConfigError(
            f"scan-pick profile schema_version must be {_SCHEMA_VERSION}"
        )
    if root.get("validated") is not True:
        raise ScanPickConfigError("scan-pick profile is missing or not validated")
    if "place_approach_height_mm" in root:
        raise ScanPickConfigError(
            "place_approach_height_mm was removed; placement releases directly "
            "at place_pose"
        )
    if "scan_z_mm" in root:
        raise ScanPickConfigError(
            "scan_z_mm was removed; scan poses use the shared working height"
        )
    missing = _REQUIRED_FIELDS.difference(root)
    if missing:
        raise ScanPickConfigError(
            "scan-pick profile is missing fields: " + ", ".join(sorted(missing))
        )
    unknown = set(root).difference(_REQUIRED_FIELDS | _OPTIONAL_FIELDS)
    if unknown:
        raise ScanPickConfigError(
            "scan-pick profile contains unknown fields: "
            + ", ".join(sorted(unknown))
        )
    place = _parse_place_pose(root.get("place_pose"), "place_pose")
    oversized_place = _parse_place_pose(
        root.get("oversized_place_pose"),
        "oversized_place_pose",
    )
    try:
        return ScanPickProfile(
            scan_x_positions_mm=tuple(root.get("scan_x_positions_mm", ())),
            scan_y_positions_mm=tuple(root.get("scan_y_positions_mm", ())),
            scan_yaw_deg=root.get("scan_yaw_deg"),
            place_pose=place,
            oversized_place_pose=oversized_place,
            max_picks_per_scan_pose=root.get("max_picks_per_scan_pose"),
            scan_settle_time_s=root.get("scan_settle_time_s", 0.0),
        )
    except (TypeError, ValueError) as exc:
        raise ScanPickConfigError(str(exc)) from exc


def _parse_place_pose(value: object, name: str) -> BaseToolTarget:
    if not isinstance(value, dict):
        raise ScanPickConfigError(f"{name} must be an object")
    if set(value) != _PLACE_FIELDS:
        raise ScanPickConfigError(
            f"{name} must contain exactly x_mm, y_mm, z_mm and yaw_deg"
        )
    try:
        return BaseToolTarget(
            value.get("x_mm"),
            value.get("y_mm"),
            value.get("z_mm"),
            value.get("yaw_deg"),
        )
    except (TypeError, ValueError) as exc:
        raise ScanPickConfigError(str(exc)) from exc


__all__ = [
    "ScanPickConfigError",
    "load_validated_scan_pick_profile",
    "parse_validated_scan_pick_profile",
]
