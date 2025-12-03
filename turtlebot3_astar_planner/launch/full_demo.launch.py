"""Launch file for full demo: Gazebo + Map Server + Path Planning + RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_astar_planner')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    algorithm = LaunchConfiguration('algorithm', default='astar')
    heuristic = LaunchConfiguration('heuristic', default='euclidean')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    map_yaml = LaunchConfiguration(
        'map_yaml',
        default=os.path.join(pkg_dir, 'maps', 'turtlebot3_world.yaml')
    )

    # 1. Launch Gazebo with TurtleBot3 world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # 2. Map server to publish the saved map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 3. Lifecycle manager to activate map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': True
        }],
        output='screen'
    )

    # 4. Static transform publisher for map -> odom
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # 5. Path planner node
    planner_node = Node(
        package='turtlebot3_astar_planner',
        executable='path_planner',
        name='path_planner',
        parameters=[{
            'algorithm': algorithm,
            'heuristic': heuristic,
            'map_yaml': map_yaml,
            'inflate_radius': 0.15,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # 6. Path follower node
    follower_node = Node(
        package='turtlebot3_astar_planner',
        executable='path_follower',
        name='path_follower',
        parameters=[{
            'lookahead_distance': 0.3,
            'linear_speed': 0.15,
            'angular_speed': 1.5,
            'goal_tolerance': 0.15,
            'angle_tolerance': 0.3,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # 7. RViz
    rviz_config = os.path.join(pkg_dir, 'rviz', 'path_planning.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Delayed launch for nodes that need Gazebo ready
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            map_server_node,
            lifecycle_manager,
            static_tf_node,
            planner_node,
            follower_node,
        ]
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
            'x_pose',
            default_value='-2.0',
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='Initial Y position'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=os.path.join(pkg_dir, 'maps', 'turtlebot3_world.yaml'),
            description='Path to map YAML file'
        ),
        gazebo_launch,
        rviz_node,
        delayed_nodes,
    ])
