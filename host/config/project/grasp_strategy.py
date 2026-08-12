"""抓取配置 loader；只有 validated=true 且字段完整才返回 GraspProfile。"""

from __future__ import annotations

import json
from pathlib import Path

from application.grasp_profile import GraspProfile, GraspYawMode


class GraspStrategyConfigError(ValueError):
    pass


def load_validated_grasp_profile(path: Path) -> GraspProfile:
    if not isinstance(path, Path):
        raise TypeError("path must be pathlib.Path")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GraspStrategyConfigError(f"cannot load grasp profile {path}: {exc}") from exc
    if not isinstance(root, dict) or root.get("schema_version") != 1:
        raise GraspStrategyConfigError("grasp profile schema_version must be 1")
    if root.get("validated") is not True:
        raise GraspStrategyConfigError("grasp profile is missing or not validated")
    try:
        return GraspProfile(
            approach_offset_mm=root.get("approach_offset_mm"),
            contact_offset_mm=root.get("contact_offset_mm"),
            retreat_offset_mm=root.get("retreat_offset_mm"),
            yaw_mode=GraspYawMode(root.get("yaw_mode")),
            fixed_yaw_deg=root.get("fixed_yaw_deg"),
            minimum_confidence=root.get("minimum_confidence"),
            maximum_observation_age_s=root.get("maximum_observation_age_s"),
            retreat_z_mm=root.get("retreat_z_mm"),
            minimum_transit_z_mm=root.get("minimum_transit_z_mm"),
        )
    except (TypeError, ValueError) as exc:
        raise GraspStrategyConfigError(str(exc)) from exc


__all__ = ["GraspStrategyConfigError", "load_validated_grasp_profile"]
