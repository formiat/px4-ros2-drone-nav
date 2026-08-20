#!/usr/bin/env bash

resolve_lidar_profile() {
  lidar_profile="${LIDAR_PROFILE:-3d}"
  lidar_profile="${lidar_profile,,}"
  case "${lidar_profile}" in
  none | 2d | 3d) ;;
  *)
    echo "LIDAR_PROFILE must be one of none, 2d, or 3d; got '${lidar_profile}'" >&2
    return 1
    ;;
  esac
}
