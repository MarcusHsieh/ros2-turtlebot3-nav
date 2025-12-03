"""Coordinate transformation utilities between world and grid frames."""

from typing import Tuple, List
import numpy as np


def world_to_grid(
    world_pos: Tuple[float, float],
    origin: List[float],
    resolution: float
) -> Tuple[int, int]:
    """
    Convert world coordinates (meters) to grid coordinates (row, col).

    The occupancy grid uses a coordinate system where:
    - Origin is at the lower-left corner of the map
    - Row increases upward (y direction)
    - Column increases rightward (x direction)

    Args:
        world_pos: World position as (x, y) in meters
        origin: Map origin as [x, y, theta] (theta typically 0)
        resolution: Grid resolution in meters per cell

    Returns:
        Grid position as (row, col)
    """
    col = int((world_pos[0] - origin[0]) / resolution)
    row = int((world_pos[1] - origin[1]) / resolution)
    return (row, col)


def grid_to_world(
    grid_pos: Tuple[int, int],
    origin: List[float],
    resolution: float
) -> Tuple[float, float]:
    """
    Convert grid coordinates (row, col) to world coordinates (meters).

    Returns the center of the grid cell.

    Args:
        grid_pos: Grid position as (row, col)
        origin: Map origin as [x, y, theta]
        resolution: Grid resolution in meters per cell

    Returns:
        World position as (x, y) in meters (center of cell)
    """
    x = origin[0] + (grid_pos[1] + 0.5) * resolution
    y = origin[1] + (grid_pos[0] + 0.5) * resolution
    return (x, y)


def path_grid_to_world(
    path: List[Tuple[int, int]],
    origin: List[float],
    resolution: float
) -> List[Tuple[float, float]]:
    """
    Convert an entire path from grid to world coordinates.

    Args:
        path: List of grid positions as [(row, col), ...]
        origin: Map origin as [x, y, theta]
        resolution: Grid resolution in meters per cell

    Returns:
        List of world positions as [(x, y), ...]
    """
    return [grid_to_world(pos, origin, resolution) for pos in path]


def simplify_path(
    path: List[Tuple[float, float]],
    tolerance: float = 0.1
) -> List[Tuple[float, float]]:
    """
    Simplify a path by removing collinear points.

    Uses the Ramer-Douglas-Peucker algorithm to reduce path complexity
    while maintaining the overall shape.

    Args:
        path: List of world positions as [(x, y), ...]
        tolerance: Maximum perpendicular distance for point removal

    Returns:
        Simplified path
    """
    if len(path) < 3:
        return path

    # Find the point farthest from the line between start and end
    start = np.array(path[0])
    end = np.array(path[-1])
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)

    if line_len < 1e-6:
        return [path[0], path[-1]]

    line_unit = line_vec / line_len

    max_dist = 0
    max_idx = 0

    for i in range(1, len(path) - 1):
        point = np.array(path[i])
        # Calculate perpendicular distance to line
        point_vec = point - start
        proj_len = np.dot(point_vec, line_unit)
        proj_point = start + proj_len * line_unit
        dist = np.linalg.norm(point - proj_point)

        if dist > max_dist:
            max_dist = dist
            max_idx = i

    # If max distance exceeds tolerance, recursively simplify
    if max_dist > tolerance:
        left = simplify_path(path[:max_idx + 1], tolerance)
        right = simplify_path(path[max_idx:], tolerance)
        return left[:-1] + right
    else:
        return [path[0], path[-1]]
