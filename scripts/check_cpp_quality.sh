#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

colcon_build_base="${COLCON_BUILD_BASE:-build}"
colcon_install_base="${COLCON_INSTALL_BASE:-install}"
colcon_log_base="${COLCON_LOG_BASE:-log}"

run_format=false
run_tidy=false
run_cppcheck=false
run_build=false
run_test=false
explicit_checks=false
all_files=false

guard_against_root_owned_workspace_writes() {
  local repo_owner_uid
  repo_owner_uid="$(stat -c '%u' "${repo_root}")"
  if [[ "${EUID}" -eq 0 && "${repo_owner_uid}" -ne 0 &&
    "${ALLOW_ROOT_WORKSPACE_WRITE:-}" != "1" ]]; then
    cat >&2 <<EOF
Refusing to run build-capable checks as root in a non-root-owned workspace.
Run through ./scripts/dev_shell.sh or docker run with:
  --user "\$(id -u):\$(id -g)"

Set ALLOW_ROOT_WORKSPACE_WRITE=1 only for intentional maintenance.
EOF
    exit 1
  fi
}

usage() {
  cat <<'EOF'
Usage: scripts/check_cpp_quality.sh [options]

Non-mutating C++ project checks.

Options:
  --all          Check all tracked project C++ files instead of changed files.
  --format      Run clang-format dry-run check.
  --tidy        Run clang-tidy when a compile database is available.
  --cppcheck    Run cppcheck on scoped project C++ files.
  --build       Run the approved colcon build command.
  --test        Run ctest against the existing package build directory.
  --no-format   Disable clang-format check.
  --no-tidy     Disable clang-tidy check.
  --no-cppcheck Disable cppcheck check.
  --no-build    Disable build.
  --no-test     Disable tests.
  -h, --help    Show this help.

With no explicit positive check options, all checks are enabled.
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --all)
      all_files=true
      ;;
    --format)
      run_format=true
      explicit_checks=true
      ;;
    --tidy)
      run_tidy=true
      explicit_checks=true
      ;;
    --cppcheck)
      run_cppcheck=true
      explicit_checks=true
      ;;
    --build)
      run_build=true
      explicit_checks=true
      ;;
    --test)
      run_test=true
      explicit_checks=true
      ;;
    --no-format)
      run_format=false
      ;;
    --no-tidy)
      run_tidy=false
      ;;
    --no-cppcheck)
      run_cppcheck=false
      ;;
    --no-build)
      run_build=false
      ;;
    --no-test)
      run_test=false
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

if [[ "${explicit_checks}" == "false" ]]; then
  run_format=true
  run_tidy=true
  run_cppcheck=true
  run_build=true
  run_test=true
fi

if [[ "${run_build}" == "true" || "${run_test}" == "true" ]]; then
  guard_against_root_owned_workspace_writes
fi

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

is_translation_unit() {
  case "$1" in
    *.c | *.cc | *.cpp | *.cxx) return 0 ;;
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

mapfile -t cpp_files < <(
  if [[ "${all_files}" == "true" ]]; then
    collect_all_cpp_files
  else
    collect_changed_cpp_files
  fi
)

mapfile -t tidy_files < <(
  printf '%s\n' "${cpp_files[@]}" | while IFS= read -r path; do
    is_translation_unit "${path}" && printf '%s\n' "${path}"
  done
)

mapfile -t cppcheck_files < <(
  printf '%s\n' "${cpp_files[@]}" | while IFS= read -r path; do
    case "${path}" in
      drone_city_nav/src/*.cpp) printf '%s\n' "${path}" ;;
    esac
  done
)

status=0

run_or_fail() {
  local label="$1"
  shift
  echo "RUN: ${label}"
  if ! "$@"; then
    echo "FAIL: ${label}" >&2
    status=1
  fi
}

if [[ "${run_format}" == "true" ]]; then
  if [[ ! -f .clang-format ]]; then
    echo "SKIP: clang-format check: .clang-format is not present"
  elif ! command -v clang-format >/dev/null 2>&1; then
    echo "SKIP: clang-format check: clang-format is not installed"
  elif [[ "${#cpp_files[@]}" -eq 0 ]]; then
    echo "SKIP: clang-format check: no scoped C++ files changed"
  else
    run_or_fail "clang-format dry-run" \
      clang-format --dry-run --Werror "${cpp_files[@]}"
  fi
fi

if [[ "${run_build}" == "true" ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "SKIP: build: colcon is not installed or ROS environment is not sourced"
  else
    run_or_fail "colcon build drone_city_nav" \
      colcon --log-base "${colcon_log_base}" build \
        --packages-select drone_city_nav --symlink-install \
        --build-base "${colcon_build_base}" \
        --install-base "${colcon_install_base}" \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  fi
fi

if [[ "${run_test}" == "true" ]]; then
  if [[ ! -d "${colcon_build_base}/drone_city_nav" ]]; then
    echo "SKIP: ctest: ${colcon_build_base}/drone_city_nav does not exist"
  elif ! command -v ctest >/dev/null 2>&1; then
    echo "SKIP: ctest: ctest is not installed"
  else
    run_or_fail "ctest drone_city_nav" \
      ctest --test-dir "${colcon_build_base}/drone_city_nav" --output-on-failure
  fi
fi

compile_db_dir=""
if [[ -f "${colcon_build_base}/compile_commands.json" ]]; then
  compile_db_dir="${colcon_build_base}"
elif [[ -f "${colcon_build_base}/drone_city_nav/compile_commands.json" ]]; then
  compile_db_dir="${colcon_build_base}/drone_city_nav"
fi

if [[ "${run_tidy}" == "true" ]]; then
  if [[ ! -f .clang-tidy ]]; then
    echo "SKIP: clang-tidy: .clang-tidy is not present"
  elif ! command -v clang-tidy >/dev/null 2>&1; then
    echo "SKIP: clang-tidy: clang-tidy is not installed"
  elif [[ -z "${compile_db_dir}" ]]; then
    echo "SKIP: clang-tidy: compile_commands.json is not available"
  elif [[ "${#tidy_files[@]}" -eq 0 ]]; then
    echo "SKIP: clang-tidy: no scoped C++ translation units changed"
  else
    run_or_fail "clang-tidy scoped files" \
      clang-tidy -p "${compile_db_dir}" "${tidy_files[@]}"
  fi
fi

if [[ "${run_cppcheck}" == "true" ]]; then
  if ! command -v cppcheck >/dev/null 2>&1; then
    echo "SKIP: cppcheck: cppcheck is not installed"
  elif [[ "${#cppcheck_files[@]}" -eq 0 ]]; then
    echo "SKIP: cppcheck: no scoped production C++ translation units changed"
  else
    echo "INFO: cppcheck gates warning/performance/portability findings; style/information checks are intentionally not enabled"
    # Actionable cppcheck findings fail the quality gate through
    # --error-exitcode=1. Informational/style-only messages such as
    # normalCheckLevelMaxBranches are intentionally outside this gate.
    run_or_fail "cppcheck scoped files" \
      cppcheck --std=c++20 --enable=warning,performance,portability \
        --inline-suppr --error-exitcode=1 --suppress=missingIncludeSystem \
        --suppress=useInitializationList -Idrone_city_nav/include \
        "${cppcheck_files[@]}"
  fi
fi

exit "${status}"
