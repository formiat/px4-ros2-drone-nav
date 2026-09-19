#!/usr/bin/env bash

resolve_lidar_profile() {
  lidar_profile="${LIDAR_PROFILE:-3d}"
  lidar_profile="${lidar_profile,,}"
  case "${lidar_profile}" in
  none | 3d) ;;
  *)
    echo "LIDAR_PROFILE must be one of none or 3d; got '${lidar_profile}'" >&2
    return 1
    ;;
  esac
}

# The camera sensor set carried beside (roadmap item 14 stages 1 to 3) or
# instead of the lidar.
resolve_camera_profile() {
  camera_profile="${CAMERA_PROFILE:-none}"
  camera_profile="${camera_profile,,}"
  case "${camera_profile}" in
  none | stereo_tof) ;;
  *)
    echo "CAMERA_PROFILE must be one of none or stereo_tof; got '${camera_profile}'" >&2
    return 1
    ;;
  esac
}
