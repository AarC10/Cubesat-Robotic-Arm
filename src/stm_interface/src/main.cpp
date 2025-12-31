#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <ioctl.h>
#include <spi.h>
#include <spidev.h>

class StmInterfaceNode : public rclcpp::Node {
public:
 StmInterfaceNode(const char* spiDevice, uint8_t mode = SPI_MODE_0,
                  uint8_t bitsPerWord = 8, uint32_t speed = 1000000)
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

    ~StmInterfaceNode() {
        if (spiDevFd >= 0) {
            close(spiDevFd);
        }

        RCLCPP_INFO(this->get_logger(), "STM Interface Node shutting down.");
    }
    
private:
    int spiDevFd;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t speed;

    static constexpr size_t BUFFER_SIZE = 256;
    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];

    void performSpiTransfer(const uint8_t* txData, uint8_t* rxData,
                            size_t length) {
        if (length > BUFFER_SIZE) {
            RCLCPP_ERROR(this->get_logger(),
                         "Transfer length exceeds buffer size.");
            throw std::runtime_error("Transfer length exceeds buffer size");
        }

        struct spi_ioc_transfer tr = {
            .tx_buf = reinterpret_cast<unsigned long>(txData),
            .rx_buf = reinterpret_cast<unsigned long>(rxData),
            .len = static_cast<uint32_t>(length),
            .speed_hz = speed,
            .bits_per_word = bitsPerWord,
        };

        int ret = ioctl(spiDevFd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to perform SPI transfer.");
        }

        // TODO: Handle receieved data
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StmInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
