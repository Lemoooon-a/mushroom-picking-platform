from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

from config.frame_transforms import (
    FixedFrameTransforms,
    frame_transforms_document_to_dict,
)
from config.robot_runtime import DEFAULT_ROBOT_RUNTIME_PATH


def write_robot_runtime_fixture(
    path: Path,
    *,
    transforms: FixedFrameTransforms | None = None,
    metadata: Mapping[str, object] | None = None,
) -> dict[str, object]:
    root = json.loads(DEFAULT_ROBOT_RUNTIME_PATH.read_text(encoding="utf-8"))
    if transforms is not None:
        root["frame_transforms"] = frame_transforms_document_to_dict(
            transforms,
            metadata=metadata,
        )
    path.write_text(
        json.dumps(root, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return root


__all__ = ["write_robot_runtime_fixture"]
