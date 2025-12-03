"""ROS2 Path Follower Node for TurtleBot3."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool


class PathFollowerNode(Node):
    """
    ROS2 node for following a planned path using pure pursuit control.

    This controller uses the pure pursuit algorithm to smoothly follow
    a path by computing steering commands based on a lookahead point.

    Subscribes to:
        /planned_path: Path to follow
        /odom: Robot odometry

    Publishes:
        /cmd_vel: Velocity commands
        /goal_reached: Navigation completion status
    """

    def __init__(self):
        super().__init__('path_follower')

        # Declare parameters
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.3)

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', qos)

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # State variables
        self.current_path = None
        self.current_waypoint_idx = 0
        self.position = None
        self.heading = None
        self.is_navigating = False

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Path follower initialized')

    def path_callback(self, msg: Path):
        """Handle new path to follow."""
        if len(msg.poses) < 2:
            self.get_logger().warn('Path too short (< 2 points), ignoring')
            return

        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.is_navigating = True

        self.get_logger().info(
            f'Received new path with {len(msg.poses)} waypoints'
        )

    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop - pure pursuit algorithm."""
        if not self.is_navigating:
            return

        if self.position is None:
            return

        if self.current_path is None:
            return

        # Get parameters
        lookahead = self.get_parameter('lookahead_distance').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        goal_tolerance = self.get_parameter('goal_tolerance').value
        angle_tolerance = self.get_parameter('angle_tolerance').value

        # Find lookahead point
        lookahead_point = self.find_lookahead_point(lookahead)

        if lookahead_point is None:
            # Reached end of path
            self.stop_robot()
            self.is_navigating = False
            self.goal_reached_pub.publish(Bool(data=True))
            self.get_logger().info('Navigation complete - goal reached!')
            return

        # Calculate distance to final goal
        final_goal = self.current_path[-1].pose.position
        dist_to_goal = math.sqrt(
            (final_goal.x - self.position[0])**2 +
            (final_goal.y - self.position[1])**2
        )

        # Check if we've reached the goal
        if dist_to_goal < goal_tolerance:
            self.stop_robot()
            self.is_navigating = False
            self.goal_reached_pub.publish(Bool(data=True))
            self.get_logger().info('Navigation complete - goal reached!')
            return

        # Calculate steering angle using pure pursuit
        dx = lookahead_point[0] - self.position[0]
        dy = lookahead_point[1] - self.position[1]

        # Angle to lookahead point
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.heading)

        # Create velocity command
        cmd = Twist()

        # If angle error is large, turn in place first
        if abs(angle_error) > angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = angular_speed * self.sign(angle_error)
            cmd.angular.z = self.clamp(cmd.angular.z, -angular_speed, angular_speed)
        else:
            # Pure pursuit steering
            # Reduce speed when turning
            speed_factor = 1.0 - min(abs(angle_error) / math.pi, 0.5)
            cmd.linear.x = linear_speed * speed_factor
            cmd.linear.x = max(0.05, cmd.linear.x)  # Minimum speed

            # Proportional angular control
            cmd.angular.z = 2.0 * angular_speed * angle_error
            cmd.angular.z = self.clamp(cmd.angular.z, -angular_speed, angular_speed)

        # Slow down near goal
        if dist_to_goal < 0.3:
            cmd.linear.x *= (dist_to_goal / 0.3)
            cmd.linear.x = max(0.03, cmd.linear.x)

        self.cmd_vel_pub.publish(cmd)

    def find_lookahead_point(self, lookahead_dist: float):
        """
        Find the lookahead point on the path.

        Returns the first point on the path that is at least
        lookahead_dist away from the robot.
        """
        if self.current_path is None or self.position is None:
            return None

        # Search from current waypoint index
        for i in range(self.current_waypoint_idx, len(self.current_path)):
            wp = self.current_path[i].pose.position
            dist = math.sqrt(
                (wp.x - self.position[0])**2 +
                (wp.y - self.position[1])**2
            )

            if dist >= lookahead_dist:
                # Update current waypoint index
                self.current_waypoint_idx = max(0, i - 1)
                return (wp.x, wp.y)

        # If no point found beyond lookahead, return last point
        if self.current_waypoint_idx < len(self.current_path) - 1:
            last = self.current_path[-1].pose.position
            return (last.x, last.y)

        return None

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max."""
        return max(min_val, min(max_val, value))

    def sign(self, value: float) -> float:
        """Return sign of value."""
        if value > 0:
            return 1.0
        elif value < 0:
            return -1.0
        return 0.0

    def stop_robot(self):
        """Send stop command to robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
