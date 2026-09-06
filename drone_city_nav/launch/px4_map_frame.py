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
