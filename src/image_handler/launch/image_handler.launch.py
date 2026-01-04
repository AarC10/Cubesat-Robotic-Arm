from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    save_directory = LaunchConfiguration("save_directory")

    return LaunchDescription([
        DeclareLaunchArgument(
            "save_directory",
            default_value="/tmp/images",
            description="Directory where raw images will be saved.",
        ),
        Node(
            package="image_handler",
            executable="image_handler_node",
            name="image_compressor",
            output="screen",
            parameters=[{"save_directory": save_directory}],
        ),
    ])
