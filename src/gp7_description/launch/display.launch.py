import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('gp7_description')
    default_urdf = os.path.join(package_share, 'urdf', 'gp7_description.urdf')
    default_rviz_config = os.path.join(package_share, 'rviz', 'gp7.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_urdf,
        description='Absolute path to robot urdf file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config,
        description='Absolute path to rviz config file'
    )

    with open(default_urdf, encoding='utf-8') as urdf_file:
        robot_description_content = urdf_file.read().replace(
            '$(find gp7_description)', package_share)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
