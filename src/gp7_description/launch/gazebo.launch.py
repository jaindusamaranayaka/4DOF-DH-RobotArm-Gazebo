import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('gp7_description')
    package_share_parent = os.path.dirname(package_share)
    world = os.path.join(package_share, 'worlds', 'gp7.world.sdf')
    urdf = os.path.join(package_share, 'urdf', 'gp7_description.urdf')

    # Ensure Ignition/GZ Gazebo can resolve model://gp7_description/meshes/...
    resource_paths = f'{package_share_parent}:{package_share}'

    env_actions = [
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=resource_paths,
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=resource_paths,
        ),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/humble/lib',
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/humble/lib',
        ),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # Bridge simulation clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    def make_robot_nodes(_context):
        with open(urdf, encoding='utf-8') as urdf_file:
            description = urdf_file.read().replace(
                '$(find gp7_description)', package_share)
        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
            output='screen',
        )
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-name', 'gp7', '-topic', 'robot_description', '-z', '0.01'],
        )
        return [state_publisher, spawn_robot]

    # The Gazebo ign_ros2_control plugin initializes the controller manager with the model.
    controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller'],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        *env_actions,
        gazebo,
        clock_bridge,
        OpaqueFunction(function=make_robot_nodes),
        controllers,
    ])
