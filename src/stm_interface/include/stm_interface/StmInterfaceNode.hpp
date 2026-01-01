#pragma once

#include <linux/spi/spidev.h>

#include <rclcpp/rclcpp.hpp>

class StmInterfaceNode : public rclcpp::Node {
   public:
    StmInterfaceNode(const char* spiDevice, uint8_t mode = SPI_MODE_0,
                     uint8_t bitsPerWord = 8, uint32_t speed = 1000000);

    ~StmInterfaceNode() override;
    
   private:
    int spiDevFd;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t speed;

    static constexpr size_t BUFFER_SIZE = 256;
    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];

    void performSpiTransfer(const uint8_t* txData, uint8_t* rxData, size_t length);
};