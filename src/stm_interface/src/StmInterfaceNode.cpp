#include "stm_interface/StmInterfaceNode.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>

StmInterfaceNode::StmInterfaceNode()
    : Node("stm_interface"),
      mode(static_cast<uint8_t>(this->declare_parameter<int>("spi_mode", SPI_MODE_0))),
      bitsPerWord(static_cast<uint8_t>(this->declare_parameter<int>("spi_bits_per_word", 8))),
      speed(static_cast<uint32_t>(this->declare_parameter<int>("spi_speed_hz", 1000000))) {
  const auto spiDevice = this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");

  spiDevFd = open(spiDevice.c_str(), O_RDWR);
  if (spiDevFd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device: %s",
                 spiDevice.c_str());
    throw std::runtime_error("Failed to open SPI device");
  }

  armStatusPublisher = this->create_publisher<arm_msgs::msg::ArmStatus>("arm_status", 10);

  ioctl(spiDevFd, SPI_IOC_WR_MODE, &mode);
  ioctl(spiDevFd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord);
  ioctl(spiDevFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

  RCLCPP_INFO(this->get_logger(), "STM Interface Node started.");
}

StmInterfaceNode::~StmInterfaceNode() {
  if (spiDevFd >= 0) {
    close(spiDevFd);
  }

  RCLCPP_INFO(this->get_logger(), "STM Interface Node shutting down.");
}

void StmInterfaceNode::receiveArmCommand(
    const arm_msgs::msg::ArmCommand::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received Arm Command: %d",
              msg->command_number);

  // Prep the packet
  SpiArmCommandPacket packet;
  packet.command_number = msg->command_number;
  packet.shoulder_yaw = msg->shoulder_yaw;
  packet.shoulder_pitch = msg->shoulder_pitch;
  packet.elbow_angle = msg->elbow_angle;
  packet.wrist_angle = msg->wrist_angle;
  packet.take_picture = msg->take_picture;

  uint8_t txBuffer[sizeof(SpiArmCommandPacket)];
  uint8_t rxBuffer[sizeof(SpiArmCommandPacket)] = {0};

  // SPI Transfer
  memcpy(txBuffer, &packet, sizeof(SpiArmCommandPacket));
  performSpiTransfer(txBuffer, rxBuffer, sizeof(SpiArmCommandPacket));

  // Parse and report status to topic
  SpiStatusPacket status;
  memcpy(&status, rxBuffer, sizeof(SpiStatusPacket));
  reportArmStatus(status);
}

void StmInterfaceNode::performSpiTransfer(const uint8_t *txData,
                                          uint8_t *rxData, size_t length) {
  if (length > BUFFER_SIZE) {
    RCLCPP_ERROR(this->get_logger(), "Transfer length exceeds buffer size.");
    throw std::runtime_error("Transfer length exceeds buffer size");
  }

  struct spi_ioc_transfer tr;
  tr.tx_buf = reinterpret_cast<unsigned long>(txData);
  tr.rx_buf = reinterpret_cast<unsigned long>(rxData);
  tr.len = static_cast<uint32_t>(length);
  tr.speed_hz = speed;

  // spidev is not thread-safe so use a mutex
  std::lock_guard<std::mutex> lock(busMutex);

  int ret = ioctl(spiDevFd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to perform SPI transfer.");
  }
}

void StmInterfaceNode::reportArmStatus(const SpiStatusPacket &status) {
  RCLCPP_INFO(
      this->get_logger(),
      "Arm Status - Fault: %d, M1 Speed: %.2f, M2 Spee: %.2f, M3 Speed: "
      "%.2f, Servo Angle: %.2f, Shoulder Yaw: %.2f, Shoulder Pitch: %.2f, "
      "Elbow Angle: %.2f, Yaw Limit Switch: %.2f, Temp1: %.2f, Temp2: %.2f",
      status.fault_status, status.m1_commanded_speed, status.m2_commanded_speed,
      status.m3_commanded_speed, status.servo_commanded_angle,
      status.shoulder_yaw, status.shoulder_pitch, status.elbow_angle,
      status.shoulder_yaw_limit_switch, status.temp1, status.temp2);

  arm_msgs::msg::ArmStatus statusMsg;
  statusMsg.fault_status = status.fault_status;
  statusMsg.m1_commanded_speed = status.m1_commanded_speed;
  statusMsg.m2_commanded_speed = status.m2_commanded_speed;
  statusMsg.m3_commanded_speed = status.m3_commanded_speed;
  statusMsg.servo_commanded_angle = status.servo_commanded_angle;
  statusMsg.shoulder_yaw = status.shoulder_yaw;
  statusMsg.shoulder_pitch = status.shoulder_pitch;
  statusMsg.elbow_angle = status.elbow_angle;
  statusMsg.shoulder_yaw_limit_switch = status.shoulder_yaw_limit_switch;
  statusMsg.temp1 = status.temp1;
  statusMsg.temp2 = status.temp2;
  armStatusPublisher->publish(statusMsg);
}