from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="gscam2",
            executable="gscam_main",
            name="camera",
            output="screen",
            parameters=[
                {
                    "gscam_config": "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGR ! appsink"
                }
            ],
        ),
        Node(
            package="image_handler",
            executable="image_handler_node",
            name="image_compressor",
            output="screen",
            parameters=[{"save_directory": "/tmp/images"}],
        ),
        Node(
            package="sensor_reader",
            executable="sensor_reader_node",
            name="sensor_reader",
            output="screen",
            parameters=[
                {
                    "read_interval_ms": 1000,
                    "adxl375_topic": "/sensor_msgs/msg/Imu",
                    "adxl375_i2c_dev": "/dev/i2c-1",
                    "adxl375_addr": 83,
                    "ina260_topic": "/sensor_msgs/msg/BatteryState",
                    "ina260_i2c_dev": "/dev/i2c-1",
                    "ina260_addr": 64,
                }
            ],
        ),
        Node(
            package="nmea_listener",
            executable="nmea_listener_node",
            name="nmea_listener",
            output="screen",
            parameters=[
                {
                    "port": "/dev/ttyS0",
                    "baud": 9600,
                    "gps_status_topic": "/gps/status",
                    "max_line_len": 512,
                }
            ],
        ),
        Node(
            package="status_accumulator",
            executable="status_accumulator_node",
            name="status_accumulator",
            output="screen",
            parameters=[
                {
                    "heartbeat_publish_interval_ms": 1000,
                    "gps_status_topic": "/gps/status",
                    "battery_state_topic": "/sensor_msgs/msg/BatteryState",
                    "sd_card_mount_point": "/mnt/sdcard",
                }
            ],
        ),
        Node(
            package="arm_commander",
            executable="arm_commander_node",
            name="arm_commander",
            output="screen",
            parameters=[
                {
                    "angle_increment": 5,
                    "timeout_interval_ms": 100,
                    "receive_topic": "/arm_target",
                    "publish_topic": "/arm_command",
                }
            ],
        ),
        Node(
            package="radio_transceiver",
            executable="radio_transceiver_node",
            name="radio_transceiver",
            output="screen",
            parameters=[
                {
                    "spi_device": "/dev/spidev0.0",
                    "gpio_chip_name": "gpiochip0",
                    "reset_gpio_line": 31,
                    "busy_gpio_line": 12,
                    "spi_speed_hz": 1000000,
                    "tx_interval_ms": 5000,
                    "rx_poll_interval_ms": 50,
                    "tx_timeout_ms": 3000,
                    "gs_timeout_sec": 60,
                    "radio_revert_sec": 30,
                    "tx_port": 9000,
                    "rx_move_arm_port": 9100,
                    "rx_set_radio_params_port": 9101,
                    "frequency_hz": 915000000,
                    "tx_power_dbm": 22,
                    "spreading_factor": 12,
                    "bandwidth_hz": 125000,
                    "coding_rate": 5,
                    "preamble_length": 8,
                    "heartbeat_topic": "heartbeat_status",
                    "arm_status_topic": "arm_status",
                    "arm_command_topic": "arm_command",
                    "arm_target_topic": "/arm_target",
                }
            ],
        ),
    ])
