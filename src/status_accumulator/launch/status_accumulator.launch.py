from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    heartbeat_publish_interval_ms = LaunchConfiguration("heartbeat_publish_interval_ms")
    gps_status_topic = LaunchConfiguration("gps_status_topic")
    battery_state_topic = LaunchConfiguration("battery_state_topic")
    sd_card_mount_point = LaunchConfiguration("sd_card_mount_point")

    return LaunchDescription([
        DeclareLaunchArgument(
            "heartbeat_publish_interval_ms",
            default_value="1000",
            description="Publish period for heartbeat status messages (milliseconds).",
        ),
        DeclareLaunchArgument(
            "gps_status_topic",
            default_value="/gps/status",
            description="Topic providing GPS status messages.",
        ),
        DeclareLaunchArgument(
            "battery_state_topic",
            default_value="/sensor_msgs/msg/BatteryState",
            description="Topic providing battery state updates.",
        ),
        DeclareLaunchArgument(
            "sd_card_mount_point",
            default_value="/mnt/sdcard",
            description="Filesystem mount point for the SD card used to compute fill percentage.",
        ),
        Node(
            package="status_accumulator",
            executable="status_accumulator_node",
            name="status_accumulator",
            output="screen",
            parameters=[
                {
                    "heartbeat_publish_interval_ms": ParameterValue(
                        heartbeat_publish_interval_ms, value_type=int
                    ),
                    "gps_status_topic": gps_status_topic,
                    "battery_state_topic": battery_state_topic,
                    "sd_card_mount_point": sd_card_mount_point,
                }
            ],
        ),
    ])
