#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

all_files=false

guard_against_root_owned_workspace_writes() {
  local repo_owner_uid
  repo_owner_uid="$(stat -c '%u' "${repo_root}")"
  if [[ "${EUID}" -eq 0 && "${repo_owner_uid}" -ne 0 &&
    "${ALLOW_ROOT_WORKSPACE_WRITE:-}" != "1" ]]; then
    cat >&2 <<EOF
Refusing to format as root in a non-root-owned workspace.
Run through ./scripts/dev_shell.sh or docker run with:
  --user "\$(id -u):\$(id -g)"

For native host formatting, prefer ./scripts/host_shell.sh ./scripts/format_cpp_changed.sh.
Set ALLOW_ROOT_WORKSPACE_WRITE=1 only for intentional maintenance.
EOF
    exit 1
  fi
}

usage() {
  cat <<'EOF'
Usage: scripts/format_cpp_changed.sh [--all]

Formats scoped project C++ files with clang-format.

Default scope is changed, staged, and untracked C++ files under:
  drone_city_nav/include
  drone_city_nav/src
  drone_city_nav/tests

Use --all only when intentionally normalizing the whole project.
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --all)
      all_files=true
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: ${arg}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

is_cpp_path() {
  case "$1" in
    drone_city_nav/include/* | drone_city_nav/src/* | drone_city_nav/tests/*)
      case "$1" in
        *.c | *.cc | *.cpp | *.cxx | *.h | *.hh | *.hpp | *.hxx) return 0 ;;
      esac
      ;;
  esac
  return 1
}

collect_all_cpp_files() {
  while IFS= read -r path; do
    is_cpp_path "${path}" && printf '%s\n' "${path}"
  done < <(git ls-files \
    'drone_city_nav/include/*' \
    'drone_city_nav/src/*' \
    'drone_city_nav/tests/*')
}

collect_changed_cpp_files() {
  {
    git diff --name-only --diff-filter=ACMRTUXB HEAD -- \
      'drone_city_nav/include/*' \
      'drone_city_nav/src/*' \
      'drone_city_nav/tests/*'
    git diff --cached --name-only --diff-filter=ACMRTUXB -- \
      'drone_city_nav/include/*' \
      'drone_city_nav/src/*' \
      'drone_city_nav/tests/*'
    git ls-files --others --exclude-standard -- \
      'drone_city_nav/include/*' \
      'drone_city_nav/src/*' \
      'drone_city_nav/tests/*'
  } | while IFS= read -r path; do
    is_cpp_path "${path}" && printf '%s\n' "${path}"
  done | sort -u
}

if [[ ! -f .clang-format ]]; then
  echo "clang-format config is missing: .clang-format" >&2
  exit 1
fi

if ! command -v clang-format >/dev/null 2>&1; then
  echo "clang-format is not installed" >&2
  exit 1
fi

guard_against_root_owned_workspace_writes

mapfile -t cpp_files < <(
  if [[ "${all_files}" == "true" ]]; then
    collect_all_cpp_files
  else
    collect_changed_cpp_files
  fi
)

if [[ "${#cpp_files[@]}" -eq 0 ]]; then
  echo "No scoped C++ files to format."
  exit 0
fi

printf 'Formatting %d C++ file(s):\n' "${#cpp_files[@]}"
printf '  %s\n' "${cpp_files[@]}"
clang-format -i "${cpp_files[@]}"
