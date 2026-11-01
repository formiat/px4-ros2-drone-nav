#!/usr/bin/env python3
"""The localization profiles of the single-vehicle launch (docs/localization.md):
which estimator owns the autopilot's truth, and, on the hybrid of the cave and
the sky (roadmap item 39), the topic the visual-inertial estimator reads the
satellite fixes from with their trust measured."""

from __future__ import annotations

LOCALIZATION_PROFILES = (
    "gnss",
    "gnss_shadow",
    "lidar_inertial",
    "visual_inertial_shadow",
    "visual_inertial",
    "hybrid",
)


def validate_localization_profile(localization: str) -> str:
    if localization not in LOCALIZATION_PROFILES:
        raise ValueError(f"unsupported localization profile: {localization}")
    return localization


def visual_inertial_overrides(localization: str, gnss_topic: str,
                              map_registration: str = "") -> dict | None:
    """The visual-inertial estimator's parameters for the profile, None where
    the estimator does not run: beside GNSS and compared (the shadow), alone
    in the autopilot's place (visual_inertial), or alone with the satellite
    fixes read from gnss_topic (hybrid). map_registration "measure" has the
    estimator read its map registration without applying it (roadmap item 40)."""
    if localization not in ("visual_inertial_shadow", "visual_inertial", "hybrid"):
        return None
    overrides: dict = {"publish_to_autopilot": localization in ("visual_inertial", "hybrid")}
    if localization == "hybrid":
        if not gnss_topic.strip():
            raise ValueError("the hybrid localization profile needs gnss_topic")
        overrides["gnss_topic"] = gnss_topic.strip()
    if map_registration.strip():
        overrides["map_registration"] = map_registration.strip()
    return overrides
