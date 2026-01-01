#pragma once

#include <linux/spi/spidev.h>

#include <rclcpp/rclcpp.hpp>

#include "arm_msgs/msg/arm_command.hpp"

class StmInterfaceNode : public rclcpp::Node {
public:
    StmInterfaceNode(const char* spiDevice, uint8_t mode = SPI_MODE_0,
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

    int spiDevFd;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t speed;

    static constexpr size_t BUFFER_SIZE = 256;
    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];

    void performSpiTransfer(const uint8_t* txData, uint8_t* rxData, size_t length);
};