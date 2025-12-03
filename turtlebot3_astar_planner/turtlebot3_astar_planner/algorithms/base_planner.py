"""Base class for path planning algorithms."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np


@dataclass(order=True)
class Node:
    """
    Node for priority queue in path planning algorithms.

    Attributes:
        f_cost: Priority value (f = g + h for A*)
        position: Grid position as (row, col)
        g_cost: Actual cost from start to this node
        h_cost: Heuristic estimate from this node to goal
        parent: Parent node for path reconstruction
    """
    f_cost: float
    position: Tuple[int, int] = field(compare=False)
    g_cost: float = field(compare=False)
    h_cost: float = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)


class BasePlanner(ABC):
    """
    Abstract base class for grid-based path planning algorithms.

    Provides common functionality for pathfinding on occupancy grids,
    including neighbor generation, validity checking, and path reconstruction.

    Attributes:
        grid: 2D numpy array where True indicates an obstacle
        resolution: Grid resolution in meters per cell
        rows: Number of rows in the grid
        cols: Number of columns in the grid
        nodes_expanded: Count of nodes expanded during planning
        path_length: Length of the found path in meters
        computation_time: Time taken for planning in seconds
    """

    def __init__(self, occupancy_grid: np.ndarray, resolution: float = 0.05):
        """
        Initialize the base planner.

        Args:
            occupancy_grid: 2D boolean array (True = obstacle, False = free)
            resolution: Grid resolution in meters per cell
        """
        self.grid = occupancy_grid
        self.resolution = resolution
        self.rows, self.cols = occupancy_grid.shape

        # 8-connected neighbors: (row_offset, col_offset)
        self.neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        # Cost for diagonal movement
        self.diagonal_cost = np.sqrt(2)

        # Metrics for algorithm comparison
        self.nodes_expanded = 0
        self.path_length = 0.0
        self.computation_time = 0.0

    @abstractmethod
    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        Find a path from start to goal.

        Args:
            start: Start position as (row, col)
            goal: Goal position as (row, col)

        Returns:
            List of grid positions from start to goal, or empty list if no path found
        """
        pass

    def is_valid(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position is valid (within bounds and not an obstacle).

        Args:
            pos: Position to check as (row, col)

        Returns:
            True if position is valid and free, False otherwise
        """
        row, col = pos
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return not self.grid[row, col]
        return False

    def get_neighbors(
        self,
        pos: Tuple[int, int]
    ) -> List[Tuple[Tuple[int, int], float]]:
        """
        Get valid neighboring positions with movement costs.

        Uses 8-connectivity (includes diagonal neighbors).
        Diagonal moves cost sqrt(2), cardinal moves cost 1.

        Args:
            pos: Current position as (row, col)

        Returns:
            List of (neighbor_position, movement_cost) tuples
        """
        valid_neighbors = []
        for dr, dc in self.neighbors:
            new_pos = (pos[0] + dr, pos[1] + dc)
            if self.is_valid(new_pos):
                # Check for corner cutting through obstacles
                if dr != 0 and dc != 0:  # Diagonal move
                    # Ensure we don't cut through obstacles
                    if not self.is_valid((pos[0] + dr, pos[1])) or \
                       not self.is_valid((pos[0], pos[1] + dc)):
                        continue
                    cost = self.diagonal_cost * self.resolution
                else:
                    cost = self.resolution
                valid_neighbors.append((new_pos, cost))
        return valid_neighbors

    def reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """
        Reconstruct the path by tracing back from goal to start.

        Args:
            node: Goal node with parent chain

        Returns:
            List of positions from start to goal
        """
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return list(reversed(path))

    def get_metrics(self) -> dict:
        """
        Get algorithm performance metrics.

        Returns:
            Dictionary with nodes_expanded, path_length, and computation_time
        """
        return {
            'nodes_expanded': self.nodes_expanded,
            'path_length': self.path_length,
            'computation_time': self.computation_time
        }
