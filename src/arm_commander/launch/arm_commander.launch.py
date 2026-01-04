from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    angle_increment = LaunchConfiguration("angle_increment")
    timeout_interval_ms = LaunchConfiguration("timeout_interval_ms")
    receive_topic = LaunchConfiguration("receive_topic")
    publish_topic = LaunchConfiguration("publish_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "angle_increment",
            default_value="5",
            description="Step size applied when moving toward target joint angles.",
        ),
        DeclareLaunchArgument(
            "timeout_interval_ms",
            default_value="100",
            description="Timer interval used to issue incremental arm commands (milliseconds).",
        ),
        DeclareLaunchArgument(
            "receive_topic",
            default_value="/arm_target",
            description="Topic on which target arm commands are received.",
        ),
        DeclareLaunchArgument(
            "publish_topic",
            default_value="/arm_command",
            description="Topic on which incremental arm commands are published.",
        ),
        Node(
            package="arm_commander",
            executable="arm_commander_node",
            name="arm_commander",
            output="screen",
            parameters=[
                {
                    "angle_increment": ParameterValue(angle_increment, value_type=int),
                    "timeout_interval_ms": ParameterValue(timeout_interval_ms, value_type=int),
                    "receive_topic": receive_topic,
                    "publish_topic": publish_topic,
                }
            ],
        ),
    ])
