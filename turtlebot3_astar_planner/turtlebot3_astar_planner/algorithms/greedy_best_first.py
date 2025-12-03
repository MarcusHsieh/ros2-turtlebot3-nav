"""Greedy Best-First Search algorithm implementation."""

import heapq
import time
from typing import List, Tuple, Callable

from .base_planner import BasePlanner, Node
from .heuristics import euclidean_distance


class GreedyBestFirstPlanner(BasePlanner):
    """
    Greedy Best-First Search path planning algorithm.

    Greedy BFS uses only the heuristic h(n) to guide the search,
    ignoring the actual path cost g(n): f(n) = h(n)

    Properties:
    - NOT guaranteed to find the optimal path
    - Can be faster than A* in some cases
    - May get stuck in local minima or find longer paths
    - Useful for comparison with A* to demonstrate the importance of g(n)

    Attributes:
        heuristic: Callable heuristic function
    """

    def __init__(
        self,
        occupancy_grid,
        resolution: float = 0.05,
        heuristic: Callable = euclidean_distance
    ):
        """
        Initialize Greedy Best-First Search planner.

        Args:
            occupancy_grid: 2D boolean array (True = obstacle)
            resolution: Grid resolution in meters per cell
            heuristic: Heuristic function (default: Euclidean distance)
        """
        super().__init__(occupancy_grid, resolution)
        self.heuristic = heuristic

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        Find path using Greedy Best-First Search.

        Note: This algorithm does NOT guarantee the optimal path.
        It may find a suboptimal path or get stuck in local minima.

        Args:
            start: Start position as (row, col)
            goal: Goal position as (row, col)

        Returns:
            List of grid positions from start to goal, empty if no path found
        """
        start_time = time.time()
        self.nodes_expanded = 0
        self.path_length = 0.0

        # Validate start and goal
        if not self.is_valid(start):
            print(f"Greedy BFS: Invalid start position {start}")
            return []
        if not self.is_valid(goal):
            print(f"Greedy BFS: Invalid goal position {goal}")
            return []

        # Initialize start node
        h_start = self.heuristic(start, goal, self.resolution)
        start_node = Node(
            f_cost=h_start,  # For Greedy BFS, f = h only
            position=start,
            g_cost=0.0,
            h_cost=h_start
        )

        # Priority queue (min-heap) - prioritized by h(n) only
        open_set = [start_node]
        # Set of visited positions
        closed_set = set()
        # Track actual path costs for metrics (not used for priority)
        g_scores = {start: 0.0}

        while open_set:
            # Get node with lowest h-cost (closest to goal by heuristic)
            current = heapq.heappop(open_set)

            # Skip if already visited
            if current.position in closed_set:
                continue

            self.nodes_expanded += 1

            # Goal reached
            if current.position == goal:
                path = self.reconstruct_path(current)
                self.computation_time = time.time() - start_time
                self.path_length = current.g_cost
                print(f"Greedy BFS: Path found! Length: {self.path_length:.3f}m, "
                      f"Nodes expanded: {self.nodes_expanded}, "
                      f"Time: {self.computation_time*1000:.2f}ms")
                return path

            closed_set.add(current.position)

            # Expand neighbors
            for neighbor_pos, move_cost in self.get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                # Calculate actual path cost (for metrics only)
                g_cost = current.g_cost + move_cost

                # For Greedy BFS, we only consider heuristic for priority
                if neighbor_pos not in g_scores:
                    g_scores[neighbor_pos] = g_cost

                    # Priority is based solely on heuristic
                    h_cost = self.heuristic(neighbor_pos, goal, self.resolution)

                    neighbor_node = Node(
                        f_cost=h_cost,  # Only heuristic for priority!
                        position=neighbor_pos,
                        g_cost=g_cost,
                        h_cost=h_cost,
                        parent=current
                    )
                    heapq.heappush(open_set, neighbor_node)

        # No path found
        self.computation_time = time.time() - start_time
        print(f"Greedy BFS: No path found! Nodes expanded: {self.nodes_expanded}, "
              f"Time: {self.computation_time*1000:.2f}ms")
        return []
