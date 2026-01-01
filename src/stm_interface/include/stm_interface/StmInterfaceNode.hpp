#pragma once

#include <linux/spi/spidev.h>

#include <rclcpp/rclcpp.hpp>

#include "arm_msgs/msg/arm_command.hpp"
#include "arm_msgs/msg/arm_status.hpp"

class StmInterfaceNode : public rclcpp::Node {
public:
  StmInterfaceNode(const char *spiDevice, uint8_t mode = SPI_MODE_0,
                   uint8_t bitsPerWord = 8, uint32_t speed = 1000000);

  ~StmInterfaceNode() override;

  void receiveArmCommand(const arm_msgs::msg::ArmCommand::SharedPtr msg);

private:
  typedef struct __attribute__((packed)) {
    uint16_t command_number;
    int16_t shoulder_yaw;
    int16_t shoulder_pitch;
    int16_t elbow_angle;
    int16_t wrist_angle;
    bool take_picture;
  } SpiArmCommandPacket;

  // TODO: Update when this is more fleshed out
  typedef struct __attribute__((packed)) {
    uint8_t fault_status;
    float m1_commanded_speed;
    float m2_commanded_speed;
    float m3_commanded_speed;
    float servo_commanded_angle;
    float shoulder_yaw;
    float shoulder_pitch;
    float elbow_angle;
    float shoulder_yaw_limit_switch;
    float temp1;
    float temp2;
  } SpiStatusPacket;

  int spiDevFd;
  uint8_t mode;
  uint8_t bitsPerWord;
  uint32_t speed;

  rclcpp::Publisher<arm_msgs::msg::ArmStatus>::SharedPtr armStatusPublisher;

  static constexpr size_t BUFFER_SIZE = 256;

  void performSpiTransfer(const uint8_t *txData, uint8_t *rxData,
                          size_t length);

  void reportArmStatus(const SpiStatusPacket &status);  
};