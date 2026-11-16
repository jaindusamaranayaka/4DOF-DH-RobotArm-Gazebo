import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gp7_description')
    gazebo_launch = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')

    # 1. Include the Gazebo simulation & controller spawners
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    # 2. Automatically launch the pick and place controller node after controllers are active
    pick_and_place_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gp7_description',
                executable='pick_and_place.py',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        sim,
        pick_and_place_node,
    ])
