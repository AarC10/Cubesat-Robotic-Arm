#include "stm_interface/StmInterfaceNode.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

StmInterfaceNode::StmInterfaceNode(const char* spiDevice,
                                   uint8_t mode,
                                   uint8_t bitsPerWord,
                                   uint32_t speed)
    : Node("stm_interface"),
      mode(mode),
      bitsPerWord(bitsPerWord),
      speed(speed) {
    spiDevFd = open(spiDevice, O_RDWR);
    if (spiDevFd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device: %s",
                     spiDevice);
        throw std::runtime_error("Failed to open SPI device");
    }

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

void StmInterfaceNode::receiveArmCommand(const arm_msgs::msg::ArmCommand::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received Arm Command: %d",
              msg->command_number);

  SpiArmCommandPacket packet;
  packet.command_number = msg->command_number;
  packet.shoulder_yaw = msg->shoulder_yaw;
  packet.shoulder_pitch = msg->shoulder_pitch;
  packet.elbow_angle = msg->elbow_angle;
  packet.wrist_angle = msg->wrist_angle;
  packet.take_picture = msg->take_picture;

  memset(txBuffer, 0, BUFFER_SIZE);
  memset(rxBuffer, 0, BUFFER_SIZE);
  memcpy(txBuffer, &packet, sizeof(SpiArmCommandPacket));

  performSpiTransfer(txBuffer, rxBuffer, sizeof(SpiArmCommandPacket));
}

void StmInterfaceNode::performSpiTransfer(const uint8_t* txData,
                                          uint8_t* rxData, size_t length) {
    if (length > BUFFER_SIZE) {
        RCLCPP_ERROR(this->get_logger(),
                     "Transfer length exceeds buffer size.");
        throw std::runtime_error("Transfer length exceeds buffer size");
    }

    struct spi_ioc_transfer tr;
    tr.tx_buf = reinterpret_cast<unsigned long>(txData);
    tr.rx_buf = reinterpret_cast<unsigned long>(rxData);
    tr.len = static_cast<uint32_t>(length);
    tr.speed_hz = speed;


    int ret = ioctl(spiDevFd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to perform SPI transfer.");
    }
}
