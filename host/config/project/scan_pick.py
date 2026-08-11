"""固定八区域扫描抓取配置加载器。"""

from __future__ import annotations

import json
from pathlib import Path

from application.motion_target import BaseToolTarget
from application.scan_pick import ScanPickProfile


class ScanPickConfigError(ValueError):
    pass


def load_validated_scan_pick_profile(path: Path) -> ScanPickProfile:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ScanPickConfigError(f"cannot load scan-pick profile {path}: {exc}") from exc
    if not isinstance(root, dict) or root.get("schema_version") != 1:
        raise ScanPickConfigError("scan-pick profile schema_version must be 1")
    if root.get("validated") is not True:
        raise ScanPickConfigError("scan-pick profile is missing or not validated")
    place = root.get("place_pose")
    if not isinstance(place, dict):
        raise ScanPickConfigError("place_pose must be an object")
    try:
        return ScanPickProfile(
            scan_x_positions_mm=tuple(root.get("scan_x_positions_mm", ())),
            scan_y_positions_mm=tuple(root.get("scan_y_positions_mm", ())),
            scan_z_mm=root.get("scan_z_mm"),
            scan_yaw_deg=root.get("scan_yaw_deg"),
            place_pose=BaseToolTarget(
                place.get("x_mm"),
                place.get("y_mm"),
                place.get("z_mm"),
                place.get("yaw_deg"),
            ),
            place_approach_height_mm=root.get("place_approach_height_mm"),
            max_picks_per_scan_pose=root.get("max_picks_per_scan_pose"),
        )
    except (TypeError, ValueError) as exc:
        raise ScanPickConfigError(str(exc)) from exc


__all__ = [
    "ScanPickConfigError",
    "load_validated_scan_pick_profile",
]
