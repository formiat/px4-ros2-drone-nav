#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
environment_id="${1:-${ENVIRONMENT_DEMO_ID:-}}"

if [[ -z "${environment_id}" ]]; then
  cat >&2 <<'EOF'
ENVIRONMENT_DEMO_ID is required.

Example:
  ENVIRONMENT_DEMO_ID=urban_circuit_practice_01 ./scripts/sim_environment_demo.sh
EOF
  exit 2
fi

python3 "${repo_root}/scripts/prepare_environment_demo.py" \
  --environment "${environment_id}"
environment_file="${repo_root}/external/environment-artifacts/derived/${environment_id}/demo/environment.env"

if [[ ! -f "${environment_file}" ]]; then
  echo "Demo environment file was not created: ${environment_file}" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${environment_file}"
if [[ -n "${SIM_DEMO_RESOURCE_PATH:-}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${SIM_DEMO_RESOURCE_PATH}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
fi

echo "Launching Gazebo spectator demo: environment=${environment_id} world=${SIM_DEMO_WORLD_NAME}"
exec gz sim -v "${GZ_VERBOSE:-3}" "${SIM_DEMO_WORLD_SDF_PATH}"
