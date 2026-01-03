#include "sensor_reader/SensorReaderNode.hpp"

SensorReaderNode::SensorReaderNode(const std::string &ina260I2cDev,
                                   const std::string &ina260Topic,
                                   uint8_t ina260Addr,
                                   const std::string &adxl375I2cDev,
                                   const std::string &adxl375Topic,
                                   uint8_t adxl375Addr, int readIntervalMs)
    : Node("sensor_reader_node"), readIntervalMs(readIntervalMs),
      adxl375(adxl375I2cDev, adxl375Addr),
      ina260(ina260I2cDev, ina260Addr, Ina260::Config{}) {
  adxl375Publisher =
      this->create_publisher<sensor_msgs::msg::Imu>(adxl375Topic, 10);
  ina260Publisher =
      this->create_publisher<sensor_msgs::msg::BatteryState>(ina260Topic, 10);

  timer = this->create_wall_timer(
      std::chrono::milliseconds(readIntervalMs),
      std::bind(&SensorReaderNode::timeoutCallback, this));

    adxl375Initialized = adxl375.init();
    ina260Initialized = ina260.init();

    if (adxl375Initialized) {
        RCLCPP_INFO(this->get_logger(), "ADXL375 initialized successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ADXL375.");
    }

    if (ina260Initialized) {
        RCLCPP_INFO(this->get_logger(), "INA260 initialized successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize INA260.");
    }
}

SensorReaderNode::~SensorReaderNode() = default;

void SensorReaderNode::timeoutCallback() {
    if (adxl375Initialized) {
        auto accelMS2 = adxl375.readMS2();
        auto imuMsg = sensor_msgs::msg::Imu();
        imuMsg.header.stamp = this->now();
        imuMsg.header.frame_id = "adxl375";

        if (accelMS2.has_value()) {
            
            imuMsg.linear_acceleration.x = accelMS2->x;
            imuMsg.linear_acceleration.y = accelMS2->y;
            imuMsg.linear_acceleration.z = accelMS2->z;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data from ADXL375.");
            imuMsg.linear_acceleration.x = NAN;
            imuMsg.linear_acceleration.y = NAN;
            imuMsg.linear_acceleration.z = NAN;
        }

        adxl375Publisher->publish(imuMsg);
    }

    if (ina260Initialized) {
        auto voltageOpt = ina260.readBusVoltage_V();
        auto currentOpt = ina260.readCurrent_A();

        auto batteryMsg = sensor_msgs::msg::BatteryState();
        batteryMsg.header.stamp = this->now();
        batteryMsg.header.frame_id = "ina260";

        if (voltageOpt.has_value() && currentOpt.has_value()) {
            batteryMsg.header.stamp = this->now();
            batteryMsg.voltage = static_cast<float>(voltageOpt.value());
            batteryMsg.current = static_cast<float>(currentOpt.value());
            ina260Publisher->publish(batteryMsg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data from INA260.");
            batteryMsg.voltage = NAN;
            batteryMsg.current = NAN;
        }
        ina260Publisher->publish(batteryMsg);
    }
}

