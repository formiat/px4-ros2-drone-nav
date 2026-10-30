#!/usr/bin/env python3
"""The launch's optional overrides (city_nav.launch.py): a launch argument left
empty leaves the node's own parameter; given, it is checked and converted
here. The rollouts of the MPPI controller (roadmap item 28 step 1) keep the
budget's proportion: the open-space count is three quarters of the full one,
as the configuration has it (8192 and 6144)."""

from __future__ import annotations

import math

import yaml


def optional_bool_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None

    normalized = value.lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False

    raise RuntimeError(
        f"Launch argument '{argument_name}' must be a boolean or empty, got '{value}'"
    )


def optional_nonnegative_float_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None
    try:
        result = float(value)
    except ValueError as error:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be numeric, got '{value}'"
        ) from error
    if result < 0.0:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be non-negative, got '{value}'"
        )
    return result


def optional_waypoint_sequence_override(context, launch_config, argument_name):
    value = launch_config.perform(context).strip()
    if not value:
        return None
    try:
        parsed = yaml.safe_load(value)
    except yaml.YAMLError as error:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must be a YAML numeric list"
        ) from error
    if not isinstance(parsed, list) or len(parsed) == 0 or len(parsed) % 3 != 0:
        raise RuntimeError(
            f"Launch argument '{argument_name}' must contain one or more x,y,z triples"
        )
    result = []
    for component in parsed:
        if isinstance(component, bool):
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain numeric components"
            )
        try:
            numeric = float(component)
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain numeric components"
            ) from error
        if not math.isfinite(numeric):
            raise RuntimeError(
                f"Launch argument '{argument_name}' must contain finite components"
            )
        result.append(numeric)
    return result


def rollouts_overrides(value):
    """The MPPI controller's parameters for a rollouts count given as text
    (MPPI_ROLLOUTS of a flight), an empty dict for none."""
    text = value.strip()
    if not text:
        return {}
    rollouts = int(text)
    if rollouts <= 0:
        raise RuntimeError(f"Launch argument 'mppi_rollouts' must be positive, got '{value}'")
    return {"rollouts": rollouts, "open_static_rollouts": rollouts * 3 // 4}


def engine_overrides(value):
    """The MPPI controller's engine given as text (MPPI_ENGINE of a flight: cuda,
    cpu or vulkan, roadmap item 28), an empty dict for the configuration's."""
    text = value.strip()
    if not text:
        return {}
    if text not in ("cuda", "cpu", "vulkan"):
        raise RuntimeError(f"Launch argument 'mppi_engine' must be cuda, cpu or vulkan, got '{value}'")
    return {"mppi_engine": text}
