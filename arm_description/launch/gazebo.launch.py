"""ROS2 launch file to spawn the robot into Gazebo using xacro -> robot_description"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('arm_description')
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'Arm.xacro'])

    model_arg = DeclareLaunchArgument('model', default_value=default_model_path,
                                      description='Path to xacro model')

    model = LaunchConfiguration('model')

    # robot_description parameter generated from xacro
    robot_description = {'robot_description': Command(['xacro ', model])}

    # Spawn entity using gazebo_ros spawn_entity.py which subscribes to robot_description topic
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Arm'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        spawn_node,
    ])
