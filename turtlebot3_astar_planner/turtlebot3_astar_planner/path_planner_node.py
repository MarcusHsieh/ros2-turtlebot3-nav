"""ROS2 Path Planner Node for TurtleBot3."""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .algorithms import AStarPlanner, DijkstraPlanner, GreedyBestFirstPlanner
from .algorithms import euclidean_distance, manhattan_distance
from .utils import MapLoader, world_to_grid, grid_to_world
from .utils.occupancy_grid_utils import occupancy_grid_to_array


class PathPlannerNode(Node):
    """
    ROS2 node for path planning using A*, Dijkstra, or Greedy Best-First.

    Uses the robot's current position from odometry as the start point,
    so each new goal plans from wherever the robot currently is.

    Subscribes to:
        /goal_pose: Goal position from RViz
        /odom: Robot odometry for current position
        /map: Occupancy grid map

    Publishes:
        /planned_path: Computed path
        /path_markers: Visualization markers
        /planner_metrics: Algorithm performance metrics
    """

    def __init__(self):
        super().__init__('path_planner')

        # Declare parameters
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('heuristic', 'euclidean')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('inflate_radius', 0.12)  # Robot radius

        # Initialize state
        self.grid = None
        self.resolution = 0.05
        self.origin = [0.0, 0.0, 0.0]
        self.map_loader = None
        self.current_position = None  # Updated from odometry
        self.goal_pose = None

        # QoS for map (transient local for late subscribers)
        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', qos_transient
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/path_markers', 10
        )
        self.metrics_pub = self.create_publisher(String, '/planner_metrics', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_transient
        )

        # Load map from file if specified
        map_yaml = self.get_parameter('map_yaml').value
        if map_yaml and os.path.exists(map_yaml):
            self.load_map_from_file(map_yaml)
        else:
            self.get_logger().info(
                'No map file specified, waiting for /map topic'
            )

        algorithm = self.get_parameter('algorithm').value
        self.get_logger().info(
            f'Path planner initialized with algorithm: {algorithm}'
        )

    def load_map_from_file(self, yaml_path: str):
        """Load map from YAML file and publish it."""
        try:
            inflate_radius = self.get_parameter('inflate_radius').value
            self.map_loader = MapLoader(yaml_path)
            self.grid = self.map_loader.get_grid_array(inflate_radius)
            self.resolution = self.map_loader.resolution
            self.origin = self.map_loader.origin

            # Publish the map
            grid_msg = self.map_loader.to_occupancy_grid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(grid_msg)

            self.get_logger().info(
                f'Loaded map from {yaml_path}: '
                f'{self.grid.shape[1]}x{self.grid.shape[0]} cells'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')

    def map_callback(self, msg: OccupancyGrid):
        """Handle map updates from /map topic."""
        self.grid, metadata = occupancy_grid_to_array(msg)
        self.resolution = metadata['resolution']
        self.origin = metadata['origin']

        self.get_logger().info(
            f'Received map: {metadata["width"]}x{metadata["height"]} cells'
        )

    def odom_callback(self, msg: Odometry):
        """Update current robot position from odometry."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg: PoseStamped):
        """Handle goal pose from RViz - plans from current position."""
        self.goal_pose = msg
        self.get_logger().info(
            f'Goal received: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )

        if self.current_position is not None and self.grid is not None:
            self.plan_and_publish()
        else:
            if self.grid is None:
                self.get_logger().warn('Cannot plan: No map available')
            if self.current_position is None:
                self.get_logger().warn(
                    'Cannot plan: No odometry received yet'
                )

    def get_planner(self):
        """Create planner based on algorithm parameter."""
        algorithm = self.get_parameter('algorithm').value
        heuristic_name = self.get_parameter('heuristic').value

        # Select heuristic function
        if heuristic_name == 'manhattan':
            heuristic = manhattan_distance
        else:
            heuristic = euclidean_distance

        # Create planner
        if algorithm == 'dijkstra':
            return DijkstraPlanner(self.grid, self.resolution)
        elif algorithm == 'greedy':
            return GreedyBestFirstPlanner(self.grid, self.resolution, heuristic)
        else:  # default: astar
            return AStarPlanner(self.grid, self.resolution, heuristic)

    def plan_and_publish(self):
        """Execute path planning and publish results."""
        if self.grid is None:
            self.get_logger().error('No map available for planning')
            return

        # Use current position from odometry as start
        start_world = self.current_position
        goal_world = (
            self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y
        )

        self.get_logger().info(
            f'Planning from ({start_world[0]:.2f}, {start_world[1]:.2f}) '
            f'to ({goal_world[0]:.2f}, {goal_world[1]:.2f})'
        )

        start_grid = world_to_grid(start_world, self.origin, self.resolution)
        goal_grid = world_to_grid(goal_world, self.origin, self.resolution)

        self.get_logger().info(
            f'Grid coordinates: {start_grid} to {goal_grid}'
        )

        # Check bounds
        if not (0 <= start_grid[0] < self.grid.shape[0] and
                0 <= start_grid[1] < self.grid.shape[1]):
            self.get_logger().error(f'Start position out of bounds: {start_grid}')
            return

        if not (0 <= goal_grid[0] < self.grid.shape[0] and
                0 <= goal_grid[1] < self.grid.shape[1]):
            self.get_logger().error(f'Goal position out of bounds: {goal_grid}')
            return

        # Check if positions are free
        if self.grid[start_grid[0], start_grid[1]]:
            self.get_logger().error('Start position is inside an obstacle!')
            return

        if self.grid[goal_grid[0], goal_grid[1]]:
            self.get_logger().error('Goal position is inside an obstacle!')
            return

        # Plan path
        planner = self.get_planner()
        path_grid = planner.plan(start_grid, goal_grid)

        if not path_grid:
            self.get_logger().warn('No path found!')
            self.publish_metrics(planner, success=False)
            return

        # Convert to world coordinates and create Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for grid_pos in path_grid:
            world_pos = grid_to_world(grid_pos, self.origin, self.resolution)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_pos[0]
            pose.pose.position.y = world_pos[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Publish path
        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published path with {len(path_msg.poses)} waypoints'
        )

        # Publish metrics
        self.publish_metrics(planner, success=True)

        # Publish visualization markers
        self.publish_path_markers(path_msg)

    def publish_metrics(self, planner, success: bool):
        """Publish algorithm performance metrics."""
        algorithm = self.get_parameter('algorithm').value
        metrics = planner.get_metrics()

        msg = String()
        msg.data = (
            f"Algorithm: {algorithm}, "
            f"Success: {success}, "
            f"Nodes expanded: {metrics['nodes_expanded']}, "
            f"Path length: {metrics['path_length']:.3f}m, "
            f"Time: {metrics['computation_time']*1000:.2f}ms"
        )
        self.metrics_pub.publish(msg)
        self.get_logger().info(msg.data)

    def publish_path_markers(self, path: Path):
        """Publish visualization markers for path in RViz."""
        markers = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header = path.header
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        # Path line strip
        line_marker = Marker()
        line_marker.header = path.header
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.03  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for pose in path.poses:
            line_marker.points.append(pose.pose.position)

        markers.markers.append(line_marker)

        # Start marker (green sphere)
        if path.poses:
            start_marker = Marker()
            start_marker.header = path.header
            start_marker.ns = 'start'
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose = path.poses[0].pose
            start_marker.scale.x = 0.1
            start_marker.scale.y = 0.1
            start_marker.scale.z = 0.1
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            markers.markers.append(start_marker)

            # Goal marker (red sphere)
            goal_marker = Marker()
            goal_marker.header = path.header
            goal_marker.ns = 'goal'
            goal_marker.id = 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose = path.poses[-1].pose
            goal_marker.scale.x = 0.1
            goal_marker.scale.y = 0.1
            goal_marker.scale.z = 0.1
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0
            markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
