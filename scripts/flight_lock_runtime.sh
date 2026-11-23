#!/usr/bin/env bash
# One flight chain a checkout at a time.
#
# Two flights of one checkout share the models' directory, the world and PX4,
# and write a run's record under one number; and every tool that flies cleans
# the checkout's simulation before and after its flight, so a second chain
# stops the first one's flight (naipi11's finding, docs/forks.md). Whatever
# flies sources this file first: the sim wrappers and the series tools on the
# host, and scripts/run_drone_nav_sim.sh in the container for a flight started
# by make alone. The first to come holds the lock until its process and the
# processes it started are gone; whoever comes while it is held is refused
# with exit status 3 before it cleans or starts anything.
#
# The lock is flock(2) on a file under log/, which the host and every
# container of the checkout share: the kernel releases it when its holders
# die, so a chain killed outright leaves nothing to take over, and no pid is
# compared across pid namespaces. What a holder starts inherits the lock
# through DRONE_GAZEBO_FLIGHT_LOCK_HELD and is not refused by its own chain.

flight_lock_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/log/.flight-lock"
if [[ -z "${DRONE_GAZEBO_FLIGHT_LOCK_HELD:-}" ]]; then
  # The directory carries .keep: the weekly pruning of log/ never removes it
  # from under a chain that flies longer than a week.
  mkdir -p "${flight_lock_dir}"
  : >> "${flight_lock_dir}/.keep"
  exec {flight_lock_fd}>> "${flight_lock_dir}/owner"
  if ! flock --nonblock "${flight_lock_fd}"; then
    echo "Refusing to fly: this checkout is flying already, started by" >&2
    sed 's/^/  /' "${flight_lock_dir}/owner" >&2
    echo "One flight a checkout at a time: wait for it to end, or stop it." >&2
    exit 3
  fi
  printf 'pid=%s host=%s started=%s command=%s\n' "$$" "$(hostname)" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$0 $*" > "${flight_lock_dir}/owner"
  export DRONE_GAZEBO_FLIGHT_LOCK_HELD=1
fi
