import math


def optional_bool(value, fallback):
    text = value.strip().lower()
    if not text:
        return fallback
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"Expected boolean launch value, got '{value}'")


def directional_hypothesis_offsets_rad(enabled, interceptor_count, target_count):
    if not enabled:
        return tuple(0.0 for _ in range(interceptor_count))
    if interceptor_count != 3 or target_count != 1:
        raise RuntimeError(
            "Directional hypotheses support only the legacy 3x1 scenario"
        )
    angle_rad = math.radians(45.0)
    return (0.0, angle_rad, -angle_rad)
