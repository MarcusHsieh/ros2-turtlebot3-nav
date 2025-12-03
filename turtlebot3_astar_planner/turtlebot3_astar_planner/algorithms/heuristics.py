"""Heuristic functions for path planning algorithms."""

import numpy as np
from typing import Tuple


def euclidean_distance(
    pos1: Tuple[int, int],
    pos2: Tuple[int, int],
    resolution: float = 1.0
) -> float:
    """
    Euclidean distance heuristic.

    Admissible for any grid (4-connected or 8-connected).

    Args:
        pos1: First position (row, col)
        pos2: Second position (row, col)
        resolution: Grid resolution in meters per cell

    Returns:
        Euclidean distance in meters
    """
    return resolution * np.sqrt(
        (pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2
    )


def manhattan_distance(
    pos1: Tuple[int, int],
    pos2: Tuple[int, int],
    resolution: float = 1.0
) -> float:
    """
    Manhattan distance heuristic.

    Admissible for 4-connected grids only.
    For 8-connected grids, this overestimates and is NOT admissible.

    Args:
        pos1: First position (row, col)
        pos2: Second position (row, col)
        resolution: Grid resolution in meters per cell

    Returns:
        Manhattan distance in meters
    """
    return resolution * (abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]))


def diagonal_distance(
    pos1: Tuple[int, int],
    pos2: Tuple[int, int],
    resolution: float = 1.0
) -> float:
    """
    Diagonal/Octile distance heuristic.

    Admissible for 8-connected grids. This is the most accurate
    heuristic for grids that allow diagonal movement.

    Args:
        pos1: First position (row, col)
        pos2: Second position (row, col)
        resolution: Grid resolution in meters per cell

    Returns:
        Diagonal distance in meters
    """
    dx = abs(pos1[0] - pos2[0])
    dy = abs(pos1[1] - pos2[1])
    # Cost: 1 for cardinal, sqrt(2) for diagonal
    return resolution * (max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy))


def zero_heuristic(
    pos1: Tuple[int, int],
    pos2: Tuple[int, int],
    resolution: float = 1.0
) -> float:
    """
    Zero heuristic - makes A* behave like Dijkstra's algorithm.

    Args:
        pos1: First position (unused)
        pos2: Second position (unused)
        resolution: Grid resolution (unused)

    Returns:
        Always returns 0.0
    """
    return 0.0
