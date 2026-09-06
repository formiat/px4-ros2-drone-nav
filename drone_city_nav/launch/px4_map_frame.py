"""PX4 local frame to map frame orientation derived from a canonical world."""

from __future__ import annotations

from typing import Any


def px4_to_map_matrix(transform: dict[str, Any]) -> tuple[float, float, float, float]:
    """Return the row-major 2x2 matrix that turns PX4 local XY into map XY.

    PX4 reports its local position in NED: X points north and Y east. Gazebo
    worlds are ENU, so north is the SDF +Y axis and east the SDF +X axis. The
    canonical world's ``map_to_sdf`` transform says which map axis feeds each
    SDF axis; inverting that axis assignment gives the map components of the
    PX4 north and east directions. A world whose map frame equals the SDF
    frame therefore needs the axis swap ``(0, 1, 1, 0)``, while a world whose
    ``map_to_sdf`` already swaps the axes keeps the identity.
    """
    source_x = transform["sdf_x_from"]
    source_y = transform["sdf_y_from"]
    scale_x = float(transform.get("sdf_x_scale", 1.0))
    scale_y = float(transform.get("sdf_y_scale", 1.0))
    sdf_x_row = (
        scale_x if source_x == "map_x" else 0.0,
        scale_x if source_x == "map_y" else 0.0,
    )
    sdf_y_row = (
        scale_y if source_y == "map_x" else 0.0,
        scale_y if source_y == "map_y" else 0.0,
    )
    return (sdf_y_row[0], sdf_x_row[0], sdf_y_row[1], sdf_x_row[1])


def map_to_sdf_swaps_axes(transform: dict[str, Any]) -> bool:
    """Return whether the map horizontal axes are exchanged in the SDF frame.

    RViz debug views use the ``gazebo_map`` fixed frame, which stands for the
    Gazebo SDF frame. A world whose ``map_to_sdf`` exchanges the axes keeps the
    legacy ``gazebo_map -> map`` rotation and its overlay conventions; a world
    whose map frame equals the SDF frame gets the identity transform.
    """
    return transform["sdf_x_from"] == "map_y"


def gazebo_aligned_map_transform_arguments(
    transform: dict[str, Any], axes_swapped: bool
) -> list[str]:
    """Return static_transform_publisher arguments for ``gazebo_map -> map``."""
    if axes_swapped:
        # Legacy visualization rotation: exchanges X/Y and flips Z. It stands
        # in for the axis exchange of ``map_to_sdf``, which is a reflection
        # and has no rigid-transform equivalent; overlays compensate Z.
        translation = ("0.0", "0.0", "0.0")
        quaternion = ("0.7071067811865476", "0.7071067811865476", "0.0", "0.0")
    else:
        translation = (
            f"{float(transform.get('sdf_x_offset_m', 0.0)):.6f}",
            f"{float(transform.get('sdf_y_offset_m', 0.0)):.6f}",
            f"{float(transform.get('sdf_z_offset_m', 0.0)):.6f}",
        )
        quaternion = ("0.0", "0.0", "0.0", "1.0")
    return [
        "--x", translation[0], "--y", translation[1], "--z", translation[2],
        "--qx", quaternion[0], "--qy", quaternion[1],
        "--qz", quaternion[2], "--qw", quaternion[3],
        "--frame-id", "gazebo_map", "--child-frame-id", "map",
    ]
