"""视觉目标的尺寸分类。"""

from __future__ import annotations

from enum import Enum


class TargetSizeClass(str, Enum):
    """视觉端已判定、供放置流程分流的目标尺寸类别。"""

    NORMAL = "normal"
    OVERSIZED = "oversized"


__all__ = ["TargetSizeClass"]
