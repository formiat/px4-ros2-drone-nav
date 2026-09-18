def optional_bool(value, fallback):
    text = value.strip().lower()
    if not text:
        return fallback
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"Expected boolean launch value, got '{value}'")
