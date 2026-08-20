#!/usr/bin/env bash

resolve_lidar_profile() {
  explicit_enable_2d_lidar=""
  if [[ -n "${ENABLE_2D_LIDAR+x}" ]]; then
    explicit_enable_2d_lidar="$(normalize_bool "${ENABLE_2D_LIDAR}")"
    case "${explicit_enable_2d_lidar}" in
    true | false) ;;
    *)
      echo "ENABLE_2D_LIDAR must be a boolean, got '${explicit_enable_2d_lidar}'" >&2
      return 1
      ;;
    esac
  fi

  lidar_profile="${LIDAR_PROFILE:-}"
  lidar_profile="${lidar_profile,,}"
  if [[ -z "${lidar_profile}" ]]; then
    if [[ "${explicit_enable_2d_lidar}" == "false" ]]; then
      lidar_profile="none"
    else
      lidar_profile="2d"
    fi
  fi
  case "${lidar_profile}" in
  none | 2d | 3d) ;;
  *)
    echo "LIDAR_PROFILE must be one of none, 2d, or 3d; got '${lidar_profile}'" >&2
    return 1
    ;;
  esac

  enable_2d_lidar="false"
  enable_3d_lidar="false"
  [[ "${lidar_profile}" == "2d" ]] && enable_2d_lidar="true"
  [[ "${lidar_profile}" == "3d" ]] && enable_3d_lidar="true"
  if [[ -n "${explicit_enable_2d_lidar}" &&
    "${explicit_enable_2d_lidar}" != "${enable_2d_lidar}" ]]; then
    echo "ENABLE_2D_LIDAR=${explicit_enable_2d_lidar} conflicts with LIDAR_PROFILE=${lidar_profile}" >&2
    return 1
  fi
}
