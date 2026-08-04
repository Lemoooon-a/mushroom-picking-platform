"""从显式 Python import path 加载项目完整 Slide-zero FK。"""

from __future__ import annotations

from importlib import import_module

from kinematics.frame_chain import SlideZeroKinematics


class FKProviderLoadError(RuntimeError):
    """完整五轴 FK provider 未配置或不满足 Protocol。"""


def load_slide_zero_kinematics(specification: str) -> SlideZeroKinematics:
    """加载 ``module:attribute``；attribute 可为实例或无参工厂。"""

    if not isinstance(specification, str) or not specification.strip():
        raise FKProviderLoadError("FK provider must be a non-empty module:attribute")
    module_name, separator, attribute_name = specification.partition(":")
    if not separator or not module_name or not attribute_name:
        raise FKProviderLoadError(
            "FK provider must use module:attribute syntax"
        )
    try:
        module = import_module(module_name)
        candidate = getattr(module, attribute_name)
    except (ImportError, AttributeError) as exc:
        raise FKProviderLoadError(
            f"cannot load FK provider {specification!r}: {exc}"
        ) from exc
    if not isinstance(candidate, SlideZeroKinematics) and callable(candidate):
        try:
            candidate = candidate()
        except Exception as exc:
            raise FKProviderLoadError(
                f"FK provider factory {specification!r} failed: {exc}"
            ) from exc
    if not isinstance(candidate, SlideZeroKinematics):
        raise FKProviderLoadError(
            f"FK provider {specification!r} must implement "
            "forward_kinematics(RobotAxisState) -> RigidTransform"
        )
    return candidate


__all__ = ["FKProviderLoadError", "load_slide_zero_kinematics"]
