"""ROS2 launch file to display the robot in RViz2 using xacro -> robot_description"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('arm_description')
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'Arm.xacro'])
    default_rviz_config = PathJoinSubstitution([pkg_share, 'launch', 'urdf.rviz'])

    model_arg = DeclareLaunchArgument('model', default_value=default_model_path,
                                      description='Path to xacro model')
    rviz_arg = DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config,
                                     description='Path to RViz config')

    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # Generate robot_description via xacro
    robot_description = {'robot_description': Command(['xacro ', model])}

    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rvizconfig],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        jsp_node,
        rsp_node,
        rviz_node,
    ])
