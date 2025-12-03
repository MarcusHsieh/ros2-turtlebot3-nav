"""Dijkstra's algorithm implementation (A* with zero heuristic)."""

from .astar import AStarPlanner
from .heuristics import zero_heuristic


class DijkstraPlanner(AStarPlanner):
    """
    Dijkstra's path planning algorithm.

    Dijkstra's algorithm is a special case of A* where the heuristic is always 0.
    This means f(n) = g(n), and the algorithm expands nodes purely based on
    their distance from the start.

    Properties:
    - Always finds the optimal (shortest) path
    - Expands more nodes than A* since it has no guidance toward the goal
    - Useful as a baseline for comparing A* performance
    """

    def __init__(self, occupancy_grid, resolution: float = 0.05):
        """
        Initialize Dijkstra planner.

        Args:
            occupancy_grid: 2D boolean array (True = obstacle)
            resolution: Grid resolution in meters per cell
        """
        super().__init__(
            occupancy_grid,
            resolution,
            heuristic=zero_heuristic,
            heuristic_weight=1.0
        )

    def plan(self, start, goal):
        """
        Find optimal path using Dijkstra's algorithm.

        Args:
            start: Start position as (row, col)
            goal: Goal position as (row, col)

        Returns:
            List of grid positions from start to goal, empty if no path found
        """
        result = super().plan(start, goal)
        # Update the log message to indicate Dijkstra's
        if result:
            print(f"Dijkstra: Path found! Length: {self.path_length:.3f}m, "
                  f"Nodes expanded: {self.nodes_expanded}, "
                  f"Time: {self.computation_time*1000:.2f}ms")
        else:
            print(f"Dijkstra: No path found! Nodes expanded: {self.nodes_expanded}, "
                  f"Time: {self.computation_time*1000:.2f}ms")
        return result
