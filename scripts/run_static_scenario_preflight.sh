#!/usr/bin/env bash
set -euo pipefail

enabled="${ENABLE_STATIC_SCENARIO_PREFLIGHT:-false}"

case "${enabled,,}" in
  ""|0|false|no|off)
    printf '%s\n' "STATIC_SCENARIO_PREFLIGHT status=skipped enabled=false"
    exit 0
    ;;
  1|true|yes|on)
    ;;
  *)
    echo "ENABLE_STATIC_SCENARIO_PREFLIGHT must be a boolean" >&2
    exit 2
    ;;
esac

if [[ "$#" -eq 0 ]]; then
  echo "static scenario preflight command is required when enabled" >&2
  exit 2
fi

printf '%s\n' "STATIC_SCENARIO_PREFLIGHT status=running enabled=true"
exec "$@"
