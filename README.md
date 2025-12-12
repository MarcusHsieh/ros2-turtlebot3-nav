# TurtleBot3 A* Path Planning

EE 148 Final Project - A* Path Planning with TurtleBot3 in Gazebo

## Overview

This project implements three path planning algorithms for TurtleBot3 navigation in Gazebo's hexagon world:

- **A\*** - Optimal pathfinding using f(n) = g(n) + h(n)
- **Dijkstra's** - Optimal pathfinding without heuristic guidance
- **Greedy Best-First Search** - Fast but non-optimal pathfinding using f(n) = h(n)

## Prerequisites

- ROS2 Humble
- TurtleBot3 packages
- Gazebo 11
- Nav2 Map Server

```bash
# Install required packages
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-lifecycle-manager
```

## Setup

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# Build the package
cd ~/ros2-turtlebot3-nav
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Run the Full Demo

```bash
cd ~/ros2-turtlebot3-nav
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch Gazebo + Path Planner + RViz
ros2 launch turtlebot3_astar_planner full_demo.launch.py algorithm:=astar
```

In RViz:
1. Click "2D Goal Pose" and click on the map to set a destination
2. Watch the robot plan a path and navigate automatically
3. Set new goals anytime - the robot plans from its current position

### Switch Algorithms

```bash
# A* (default) - optimal, uses heuristic
ros2 launch turtlebot3_astar_planner full_demo.launch.py algorithm:=astar

# Dijkstra's - optimal, no heuristic (explores more nodes)
ros2 launch turtlebot3_astar_planner full_demo.launch.py algorithm:=dijkstra

# Greedy Best-First - fast but non-optimal
ros2 launch turtlebot3_astar_planner full_demo.launch.py algorithm:=greedy
```

### Monitor Metrics

```bash
ros2 topic echo /planner_metrics
```

Example output:
```
data: "Algorithm: astar, Success: True, Nodes expanded: 342, Path length: 3.45m, Time: 15.2ms"
```

## Algorithm Comparison

| Algorithm | Cost Function | Optimal | Nodes Expanded |
|-----------|---------------|---------|----------------|
| A* | f(n) = g(n) + h(n) | Yes | Moderate |
| Dijkstra | f(n) = g(n) | Yes | High |
| Greedy BFS | f(n) = h(n) | No | Low |

## Project Structure

```
turtlebot3_astar_planner/
├── turtlebot3_astar_planner/
│   ├── algorithms/              # Path planning algorithms
│   │   ├── astar.py             # A* implementation
│   │   ├── dijkstra.py          # Dijkstra's implementation
│   │   ├── greedy_best_first.py # Greedy Best-First implementation
│   │   ├── base_planner.py      # Base class for planners
│   │   └── heuristics.py        # Heuristic functions
│   ├── utils/                   # Utility functions
│   │   ├── occupancy_grid_utils.py
│   │   └── coordinate_transforms.py
│   ├── path_planner_node.py     # Main planning node
│   └── path_follower_node.py    # Pure pursuit path follower
├── launch/
│   ├── full_demo.launch.py      # Main demo launch file
│   ├── gazebo_world.launch.py   # Gazebo only
│   └── path_planning.launch.py  # Planning without Gazebo
├── rviz/
│   └── path_planning.rviz       # RViz configuration
└── maps/
    ├── turtlebot3_world.yaml    # Map metadata
    └── turtlebot3_world.pgm     # Occupancy grid image
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planned_path` | nav_msgs/Path | Computed path |
| `/goal_pose` | geometry_msgs/PoseStamped | Goal from RViz |
| `/odom` | nav_msgs/Odometry | Robot odometry (current position) |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/planner_metrics` | std_msgs/String | Algorithm performance metrics |
| `/path_markers` | visualization_msgs/MarkerArray | Path visualization |
| `/goal_reached` | std_msgs/Bool | Goal completion status |

## Parameters

### Path Planner Node
- `algorithm`: Planning algorithm (`astar`, `dijkstra`, `greedy`)
- `heuristic`: Heuristic function (`euclidean`, `manhattan`)
- `inflate_radius`: Obstacle inflation radius (default: 0.15m)

### Path Follower Node
- `lookahead_distance`: Pure pursuit lookahead (default: 0.3m)
- `linear_speed`: Maximum linear velocity (default: 0.15 m/s)
- `angular_speed`: Maximum angular velocity (default: 1.5 rad/s)
- `goal_tolerance`: Distance to consider goal reached (default: 0.15m)

## Images
![alt text](media/gazebo-1.png)
![alt text](media/rviz2-1.png)


## License

MIT License