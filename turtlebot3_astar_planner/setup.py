import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_astar_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.edu',
    description='A* Path Planning for TurtleBot3 - EE148 Final Project',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_planner = turtlebot3_astar_planner.path_planner_node:main',
            'path_follower = turtlebot3_astar_planner.path_follower_node:main',
        ],
    },
)
