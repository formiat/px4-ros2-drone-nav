#!/usr/bin/env python3
"""Resolve the navigation lidar profile and the vehicle model identities."""

from __future__ import annotations

from typing import Any


LIDAR_PROFILES = ("3d",)
DEFAULT_LIDAR_PROFILE = "3d"


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
    """Return the PX4 model target and exact Gazebo entity name of a vehicle."""
    validate_lidar_profile(lidar_profile)
    return px4_model_target, gazebo_model_name


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
