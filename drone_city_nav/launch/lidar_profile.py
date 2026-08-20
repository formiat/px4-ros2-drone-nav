#!/usr/bin/env python3
"""Resolve mutually exclusive navigation lidar profiles and model identities."""

from __future__ import annotations

from typing import Any


LIDAR_PROFILES = ("none", "2d", "3d")
_MODEL_2D_TOKEN = "x500_lidar_2d"
_MODEL_3D_TOKEN = "x500_lidar_3d"


def validate_lidar_profile(value: str) -> str:
    profile = value.strip().lower()
    if profile not in LIDAR_PROFILES:
        raise ValueError(
            f"lidar profile must be one of {', '.join(LIDAR_PROFILES)}, got '{value}'"
        )
    return profile


def resolve_model_identity(
    px4_model_target: str, gazebo_model_name: str, lidar_profile: str
) -> tuple[str, str]:
    """Return the profile-specific model target and exact Gazebo entity name."""
    profile = validate_lidar_profile(lidar_profile)
    if profile != "3d":
        return px4_model_target, gazebo_model_name
    if _MODEL_2D_TOKEN not in px4_model_target:
        raise ValueError(
            "3D lidar profile requires an x500_lidar_2d-compatible PX4 model target"
        )
    if _MODEL_2D_TOKEN not in gazebo_model_name:
        raise ValueError(
            "3D lidar profile requires an x500_lidar_2d-compatible Gazebo model name"
        )
    return (
        px4_model_target.replace(_MODEL_2D_TOKEN, _MODEL_3D_TOKEN, 1),
        gazebo_model_name.replace(_MODEL_2D_TOKEN, _MODEL_3D_TOKEN, 1),
    )


def apply_profile_to_vehicle(
    vehicle: dict[str, Any], lidar_profile: str
) -> dict[str, Any]:
    resolved = dict(vehicle)
    target, model_name = resolve_model_identity(
        str(vehicle["px4_model_target"]),
        str(vehicle["gazebo_model_name"]),
        lidar_profile,
    )
    resolved["px4_model_target"] = target
    resolved["gazebo_model_name"] = model_name
    return resolved
