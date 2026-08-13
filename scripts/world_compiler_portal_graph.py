"""Derive semantic passage topology from voxelized physical occupancy."""

from __future__ import annotations

import hashlib
import heapq
import math
from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Iterable


Cell = tuple[int, int, int]
Direction = tuple[int, int, int]

AXIAL_DIRECTIONS: tuple[Direction, ...] = (
    (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)
)
HORIZONTAL_DIRECTIONS: tuple[Direction, ...] = AXIAL_DIRECTIONS[:4]
NEIGHBOR_DIRECTIONS: tuple[Direction, ...] = tuple(
    (dx, dy, dz)
    for dz in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


@dataclass(frozen=True)
class DerivedPortal:
    id: str
    center: tuple[float, float, float]
    outward_normal: tuple[float, float, float]
    opening_polygon: tuple[tuple[float, float, float], ...]
    cells: frozenset[Cell]


@dataclass(frozen=True)
class DerivedTraversalEdge:
    id: str
    region_id: str
    entry_portal_id: str
    exit_portal_id: str
    centerline: tuple[tuple[float, float, float], ...]
    min_z_m: float
    max_z_m: float
    width_m: float
    height_m: float
    minimum_clearance_m: float
    speed_limit_mps: float


@dataclass(frozen=True)
class DerivedPassageRegion:
    id: str
    portals: tuple[DerivedPortal, ...]


@dataclass(frozen=True)
class DerivedPortalGraph:
    regions: tuple[DerivedPassageRegion, ...]
    traversal_edges: tuple[DerivedTraversalEdge, ...]


def _add(cell: Cell, direction: Direction) -> Cell:
    return (cell[0] + direction[0], cell[1] + direction[1],
            cell[2] + direction[2])


def _cell_center(cell: Cell, origin: tuple[float, float, float],
                 resolution: float) -> tuple[float, float, float]:
    return tuple(origin[axis] + (cell[axis] + 0.5) * resolution for axis in range(3))


def _distance_to_interval(value: float, minimum: float, maximum: float) -> float:
    return max(minimum - value, 0.0, value - maximum)


def _capsule_clear(position: tuple[float, float, float],
                   column_masks: dict[tuple[int, int], int],
                   dimensions: tuple[int, int, int],
                   origin: tuple[float, float, float], resolution: float,
                   radius_m: float, lower_extent_m: float,
                   upper_extent_m: float) -> bool:
    minimum_x = max(0, math.floor((position[0] - radius_m - origin[0]) /
                                  resolution) - 1)
    maximum_x = min(dimensions[0] - 1,
                    math.floor((position[0] + radius_m - origin[0]) /
                               resolution) + 1)
    minimum_y = max(0, math.floor((position[1] - radius_m - origin[1]) /
                                  resolution) - 1)
    maximum_y = min(dimensions[1] - 1,
                    math.floor((position[1] + radius_m - origin[1]) /
                               resolution) + 1)
    radius_squared = radius_m * radius_m
    segment_minimum_z = position[2] - lower_extent_m
    segment_maximum_z = position[2] + upper_extent_m
    for y in range(minimum_y, maximum_y + 1):
        box_minimum_y = origin[1] + y * resolution
        dy = _distance_to_interval(position[1], box_minimum_y,
                                   box_minimum_y + resolution)
        for x in range(minimum_x, maximum_x + 1):
            occupied = column_masks.get((x, y), 0)
            if occupied == 0:
                continue
            box_minimum_x = origin[0] + x * resolution
            dx = _distance_to_interval(position[0], box_minimum_x,
                                       box_minimum_x + resolution)
            horizontal_squared = dx * dx + dy * dy
            if horizontal_squared > radius_squared:
                continue
            vertical_radius = math.sqrt(max(0.0,
                                            radius_squared - horizontal_squared))
            query_minimum_z = segment_minimum_z - vertical_radius
            query_maximum_z = segment_maximum_z + vertical_radius
            minimum_z = max(0, math.ceil((query_minimum_z - origin[2]) /
                                         resolution) - 1)
            maximum_z = min(dimensions[2] - 1,
                            math.floor((query_maximum_z - origin[2]) /
                                       resolution))
            if maximum_z < minimum_z:
                continue
            mask = ((1 << (maximum_z - minimum_z + 1)) - 1) << minimum_z
            if occupied & mask:
                return False
    return True


def _stable_id(prefix: str, values: Iterable[object]) -> str:
    payload = "|".join(str(value) for value in values).encode("ascii")
    return f"{prefix}_{hashlib.sha256(payload).hexdigest()[:12]}"


def _roofed_free_cells(column_masks: dict[tuple[int, int], int],
                       depth: int) -> set[Cell]:
    cells: set[Cell] = set()
    for (x, y), occupied in column_masks.items():
        occupied_above = False
        for z in range(depth - 1, -1, -1):
            if occupied & (1 << z):
                occupied_above = True
            elif occupied_above:
                cells.add((x, y, z))
    return cells


def _connected_components(cells: set[Cell]) -> list[set[Cell]]:
    remaining = set(cells)
    components: list[set[Cell]] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        component = {start}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for direction in AXIAL_DIRECTIONS:
                neighbor = _add(current, direction)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def _is_occupied(cell: Cell, column_masks: dict[tuple[int, int], int],
                 dimensions: tuple[int, int, int]) -> bool:
    if any(cell[axis] < 0 or cell[axis] >= dimensions[axis] for axis in range(3)):
        return True
    return bool(column_masks.get((cell[0], cell[1]), 0) & (1 << cell[2]))


def _portal_boundary_cells(component: set[Cell], roofed: set[Cell],
                           column_masks: dict[tuple[int, int], int],
                           dimensions: tuple[int, int, int]) -> dict[Direction, set[Cell]]:
    boundaries: dict[Direction, set[Cell]] = {}
    for cell in component:
        for direction in HORIZONTAL_DIRECTIONS:
            neighbor = _add(cell, direction)
            if neighbor in roofed or _is_occupied(neighbor, column_masks, dimensions):
                continue
            boundaries.setdefault(direction, set()).add(cell)
    return boundaries


def _cluster_face_cells(cells: set[Cell]) -> list[set[Cell]]:
    remaining = set(cells)
    clusters: list[set[Cell]] = []
    while remaining:
        start = min(remaining)
        remaining.remove(start)
        cluster = {start}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            for direction in NEIGHBOR_DIRECTIONS:
                neighbor = _add(current, direction)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    cluster.add(neighbor)
                    queue.append(neighbor)
        clusters.append(cluster)
    return clusters


def _portal_polygon(cells: set[Cell], normal: Direction,
                    origin: tuple[float, float, float], resolution: float
                    ) -> tuple[tuple[float, float, float], ...]:
    minimum = [min(cell[axis] for cell in cells) for axis in range(3)]
    maximum = [max(cell[axis] for cell in cells) + 1 for axis in range(3)]
    bounds_min = [origin[axis] + minimum[axis] * resolution for axis in range(3)]
    bounds_max = [origin[axis] + maximum[axis] * resolution for axis in range(3)]
    if normal[0] != 0:
        plane = bounds_max[0] if normal[0] > 0 else bounds_min[0]
        return ((plane, bounds_min[1], bounds_min[2]),
                (plane, bounds_max[1], bounds_min[2]),
                (plane, bounds_max[1], bounds_max[2]),
                (plane, bounds_min[1], bounds_max[2]))
    plane = bounds_max[1] if normal[1] > 0 else bounds_min[1]
    return ((bounds_min[0], plane, bounds_min[2]),
            (bounds_max[0], plane, bounds_min[2]),
            (bounds_max[0], plane, bounds_max[2]),
            (bounds_min[0], plane, bounds_max[2]))


def _make_portals(component: set[Cell], roofed: set[Cell], region_id: str,
                  column_masks: dict[tuple[int, int], int],
                  dimensions: tuple[int, int, int],
                  origin: tuple[float, float, float], resolution: float,
                  minimum_opening_area_m2: float) -> list[DerivedPortal]:
    portals: list[DerivedPortal] = []
    boundaries = _portal_boundary_cells(component, roofed, column_masks, dimensions)
    for normal, boundary in sorted(boundaries.items()):
        for cells in _cluster_face_cells(boundary):
            polygon = _portal_polygon(cells, normal, origin, resolution)
            first = polygon[0]
            second = polygon[1]
            fourth = polygon[3]
            area = math.dist(first, second) * math.dist(first, fourth)
            if area + 1.0e-9 < minimum_opening_area_m2:
                continue
            center = tuple(sum(point[axis] for point in polygon) / len(polygon)
                           for axis in range(3))
            geometry_key = (*normal, *(min(cell[axis] for cell in cells)
                                        for axis in range(3)),
                            *(max(cell[axis] for cell in cells)
                              for axis in range(3)), len(cells))
            portal_id = f"{region_id}:{_stable_id('portal', geometry_key)}"
            portals.append(DerivedPortal(
                id=portal_id,
                center=center,
                outward_normal=tuple(float(value) for value in normal),
                opening_polygon=polygon,
                cells=frozenset(cells),
            ))
    return sorted(portals, key=lambda portal: portal.id)


def _clearance_cells(component: set[Cell]) -> dict[Cell, int]:
    clearance: dict[Cell, int] = {}
    queue: deque[Cell] = deque()
    for cell in component:
        if any(_add(cell, direction) not in component for direction in AXIAL_DIRECTIONS):
            clearance[cell] = 0
            queue.append(cell)
    while queue:
        current = queue.popleft()
        next_clearance = clearance[current] + 1
        for direction in AXIAL_DIRECTIONS:
            neighbor = _add(current, direction)
            if neighbor in component and neighbor not in clearance:
                clearance[neighbor] = next_clearance
                queue.append(neighbor)
    return clearance


def _portal_anchor(portal: DerivedPortal, traversable: set[Cell],
                   clearance: dict[Cell, int],
                   origin: tuple[float, float, float], resolution: float) -> Cell:
    candidates = portal.cells & traversable
    if not candidates:
        candidates = traversable
    return min(
        candidates,
        key=lambda cell: (
            math.dist(_cell_center(cell, origin, resolution), portal.center),
            -clearance.get(cell, 0), cell,
        ),
    )


def _central_path(component: set[Cell], start: Cell, goal: Cell,
                  clearance: dict[Cell, int]) -> list[Cell]:
    queue: list[tuple[float, float, Cell]] = [(math.dist(start, goal), 0.0, start)]
    cost = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost > cost[current] + 1.0e-9:
            continue
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            return path
        for direction in AXIAL_DIRECTIONS:
            neighbor = _add(current, direction)
            if neighbor not in component:
                continue
            centrality_tiebreak = 0.01 / (1.0 + clearance.get(neighbor, 0))
            candidate_cost = current_cost + 1.0 + centrality_tiebreak
            if candidate_cost + 1.0e-9 >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = candidate_cost
            parent[neighbor] = current
            heuristic = math.dist(neighbor, goal)
            heapq.heappush(queue, (candidate_cost + heuristic, candidate_cost, neighbor))
    raise ValueError("derived passage component has disconnected portal anchors")


def _segment_clear(first: Cell, second: Cell, component: set[Cell],
                   column_masks: dict[tuple[int, int], int],
                   dimensions: tuple[int, int, int],
                   origin: tuple[float, float, float], resolution: float,
                   radius_m: float, lower_extent_m: float,
                   upper_extent_m: float, sweep_step_m: float) -> bool:
    first_position = _cell_center(first, origin, resolution)
    second_position = _cell_center(second, origin, resolution)
    length = math.dist(first_position, second_position)
    sample_count = max(1, math.ceil(length / sweep_step_m))
    for sample in range(sample_count + 1):
        ratio = sample / sample_count
        position = tuple(first_position[axis] +
                         ratio * (second_position[axis] - first_position[axis])
                         for axis in range(3))
        cell = tuple(math.floor((position[axis] - origin[axis]) / resolution)
                     for axis in range(3))
        if cell not in component or not _capsule_clear(
                position, column_masks, dimensions, origin, resolution,
                radius_m, lower_extent_m, upper_extent_m):
            return False
    return True


def _shortcut_path(path: list[Cell], component: set[Cell],
                   column_masks: dict[tuple[int, int], int],
                   dimensions: tuple[int, int, int],
                   origin: tuple[float, float, float], resolution: float,
                   radius_m: float, lower_extent_m: float,
                   upper_extent_m: float, sweep_step_m: float) -> list[Cell]:
    result = [path[0]]
    index = 0
    while index + 1 < len(path):
        candidate = len(path) - 1
        while candidate > index + 1 and not _segment_clear(
                path[index], path[candidate], component, column_masks, dimensions,
                origin, resolution, radius_m, lower_extent_m, upper_extent_m,
                sweep_step_m):
            candidate -= 1
        result.append(path[candidate])
        index = candidate
    return result


def _portal_width(portal: DerivedPortal) -> float:
    polygon = portal.opening_polygon
    horizontal_edges = [math.dist(polygon[index], polygon[(index + 1) % len(polygon)])
                        for index in range(len(polygon))
                        if abs(polygon[index][2] -
                               polygon[(index + 1) % len(polygon)][2]) < 1.0e-9]
    return max(horizontal_edges)


def _portal_vertical_bounds(portal: DerivedPortal) -> tuple[float, float]:
    heights = [point[2] for point in portal.opening_polygon]
    return min(heights), max(heights)


def derive_portal_graph(column_masks: dict[tuple[int, int], int],
                        dimensions: tuple[int, int, int],
                        origin: tuple[float, float, float], resolution: float,
                        minimum_opening_area_m2: float = 4.0,
                        speed_limit_mps: float = 10.0,
                        validation_radius_m: float = 0.82,
                        validation_lower_extent_m: float = 0.23,
                        validation_upper_extent_m: float = 0.35,
                        validation_sweep_step_m: float = 0.25) -> DerivedPortalGraph:
    """Build passage regions and traversal edges using only occupied voxels."""
    roofed = _roofed_free_cells(column_masks, dimensions[2])
    region_records: list[tuple[set[Cell], DerivedPassageRegion]] = []
    for component in _connected_components(roofed):
        component_bounds = tuple(
            value
            for axis in range(3)
            for value in (min(cell[axis] for cell in component),
                          max(cell[axis] for cell in component))
        )
        temporary_id = _stable_id("passage_region", (*component_bounds, len(component)))
        portals = _make_portals(component, roofed, temporary_id, column_masks,
                                dimensions, origin, resolution,
                                minimum_opening_area_m2)
        if len(portals) >= 2:
            region_records.append((component, DerivedPassageRegion(
                id=temporary_id, portals=tuple(portals))))

    region_records.sort(key=lambda item: item[1].id)
    edges: list[DerivedTraversalEdge] = []
    for component, region in region_records:
        traversable = {
            cell for cell in component
            if _capsule_clear(_cell_center(cell, origin, resolution), column_masks,
                              dimensions, origin, resolution, validation_radius_m,
                              validation_lower_extent_m,
                              validation_upper_extent_m)
        }
        if not traversable:
            continue
        clearance = _clearance_cells(traversable)
        for first, second in combinations(region.portals, 2):
            first_anchor = _portal_anchor(first, traversable, clearance, origin,
                                          resolution)
            second_anchor = _portal_anchor(second, traversable, clearance, origin,
                                           resolution)
            try:
                raw_path = _central_path(traversable, first_anchor, second_anchor,
                                         clearance)
            except ValueError:
                continue
            cell_path = _shortcut_path(
                raw_path, component, column_masks, dimensions, origin, resolution,
                validation_radius_m, validation_lower_extent_m,
                validation_upper_extent_m, validation_sweep_step_m,
            )
            points = (first.center,
                      *(_cell_center(cell, origin, resolution) for cell in cell_path),
                      second.center)
            min_z_first, max_z_first = _portal_vertical_bounds(first)
            min_z_second, max_z_second = _portal_vertical_bounds(second)
            min_z = max(min_z_first, min_z_second)
            max_z = min(max_z_first, max_z_second)
            width = min(_portal_width(first), _portal_width(second))
            height = max_z - min_z
            if width <= 0.0 or height <= 0.0:
                continue
            edge_id = f"{region.id}:edge_{first.id.rsplit('_', 1)[-1]}_" \
                      f"{second.id.rsplit('_', 1)[-1]}"
            edges.append(DerivedTraversalEdge(
                id=edge_id,
                region_id=region.id,
                entry_portal_id=first.id,
                exit_portal_id=second.id,
                centerline=points,
                min_z_m=min_z,
                max_z_m=max_z,
                width_m=width,
                height_m=height,
                minimum_clearance_m=0.5 * min(width, height),
                speed_limit_mps=speed_limit_mps,
            ))
    return DerivedPortalGraph(
        regions=tuple(region for _, region in region_records),
        traversal_edges=tuple(sorted(edges, key=lambda edge: edge.id)),
    )
