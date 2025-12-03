"""A* path planning algorithm implementation."""

import heapq
import time
from typing import List, Tuple, Callable

from .base_planner import BasePlanner, Node
from .heuristics import euclidean_distance


class AStarPlanner(BasePlanner):
    """
    A* path planning algorithm.

    A* finds the optimal path by combining actual path cost g(n) with
    heuristic estimate h(n): f(n) = g(n) + h(n)

    The algorithm is guaranteed to find the optimal path when using an
    admissible heuristic (one that never overestimates the true cost).

    Attributes:
        heuristic: Callable heuristic function
        weight: Weight for heuristic (1.0 for standard A*, >1.0 for weighted A*)
    """

    def __init__(
        self,
        occupancy_grid,
        resolution: float = 0.05,
        heuristic: Callable = euclidean_distance,
        heuristic_weight: float = 1.0
    ):
        """
        Initialize A* planner.

        Args:
            occupancy_grid: 2D boolean array (True = obstacle)
            resolution: Grid resolution in meters per cell
            heuristic: Heuristic function (default: Euclidean distance)
            heuristic_weight: Weight for heuristic (default: 1.0)
        """
        super().__init__(occupancy_grid, resolution)
        self.heuristic = heuristic
        self.weight = heuristic_weight

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """
        Find optimal path using A* algorithm.

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
            print(f"A*: Invalid start position {start}")
            return []
        if not self.is_valid(goal):
            print(f"A*: Invalid goal position {goal}")
            return []

        # Initialize start node
        h_start = self.weight * self.heuristic(start, goal, self.resolution)
        start_node = Node(
            f_cost=h_start,
            position=start,
            g_cost=0.0,
            h_cost=h_start
        )

        # Priority queue (min-heap)
        open_set = [start_node]
        # Set of visited positions
        closed_set = set()
        # Best g-cost found for each position
        g_scores = {start: 0.0}

        while open_set:
            # Get node with lowest f-cost
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
                print(f"A*: Path found! Length: {self.path_length:.3f}m, "
                      f"Nodes expanded: {self.nodes_expanded}, "
                      f"Time: {self.computation_time*1000:.2f}ms")
                return path

            closed_set.add(current.position)

            # Expand neighbors
            for neighbor_pos, move_cost in self.get_neighbors(current.position):
                if neighbor_pos in closed_set:
                    continue

                tentative_g = current.g_cost + move_cost

                # Skip if we've found a better path to this neighbor
                if neighbor_pos in g_scores and tentative_g >= g_scores[neighbor_pos]:
                    continue

                # Update best g-score
                g_scores[neighbor_pos] = tentative_g

                # Calculate heuristic and f-cost
                h_cost = self.weight * self.heuristic(
                    neighbor_pos, goal, self.resolution
                )
                f_cost = tentative_g + h_cost

                # Create neighbor node and add to open set
                neighbor_node = Node(
                    f_cost=f_cost,
                    position=neighbor_pos,
                    g_cost=tentative_g,
                    h_cost=h_cost,
                    parent=current
                )
                heapq.heappush(open_set, neighbor_node)

        # No path found
        self.computation_time = time.time() - start_time
        print(f"A*: No path found! Nodes expanded: {self.nodes_expanded}, "
              f"Time: {self.computation_time*1000:.2f}ms")
        return []
