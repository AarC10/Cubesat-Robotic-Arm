# camera_unified.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    platform = LaunchConfiguration("platform")

    return LaunchDescription([
        DeclareLaunchArgument(
            "platform",
            default_value="jetson",
            description="Target platform: 'jetson' or 'pi'",
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", platform, "' == 'jetson'"])),
            package="gscam",
            executable="gscam_node",
            name="camera",
            output="screen",
            parameters=[{
                "gscam_config": (
                "nvarguscamerasrc sensor-id=0 ! "
                "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
                "nvvidconv ! "
                "video/x-raw,format=BGRx ! "
                "videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink"
                )
            }],
           remappings=[
                ("image", "image_raw"),
            ],
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", platform, "' == 'pi'"])),
            package="camera_ros",
            executable="camera_node",
            name="camera",
            output="screen",
            parameters=[{
                "camera_name": "camera",
                "width": 1280,
                "height": 720,
                "frame_rate": 30,
            }],
        ),
    ])
