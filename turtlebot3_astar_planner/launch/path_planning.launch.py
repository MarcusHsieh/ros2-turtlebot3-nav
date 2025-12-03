"""Launch file for path planning nodes."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_astar_planner')

    # Launch arguments
    algorithm = LaunchConfiguration('algorithm', default='astar')
    heuristic = LaunchConfiguration('heuristic', default='euclidean')
    map_yaml = LaunchConfiguration(
        'map_yaml',
        default=os.path.join(pkg_dir, 'maps', 'turtlebot3_world.yaml')
    )

    # Path planner node
    planner_node = Node(
        package='turtlebot3_astar_planner',
        executable='path_planner',
        name='path_planner',
        parameters=[{
            'algorithm': algorithm,
            'heuristic': heuristic,
            'map_yaml': map_yaml,
            'inflate_radius': 0.12,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # Path follower node
    follower_node = Node(
        package='turtlebot3_astar_planner',
        executable='path_follower',
        name='path_follower',
        parameters=[{
            'lookahead_distance': 0.3,
            'linear_speed': 0.15,
            'angular_speed': 1.5,
            'goal_tolerance': 0.1,
            'angle_tolerance': 0.3,
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm',
            default_value='astar',
            description='Planning algorithm: astar, dijkstra, or greedy'
        ),
        DeclareLaunchArgument(
            'heuristic',
            default_value='euclidean',
            description='Heuristic function: euclidean or manhattan'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Path to map YAML file'
        ),
        planner_node,
        follower_node,
    ])
