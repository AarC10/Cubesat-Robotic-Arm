#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <ioctl.h>
#include <spi.h>

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
        RCLCPP_INFO(this->get_logger(), "STM Interface Node shutting down.");
    }
    
private:
    int spiDevFd;
    uint8_t mode;
    uint8_t bitsPerWord;
    uint32_t speed;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StmInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
