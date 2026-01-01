#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <ioctl.h>
#include <spi.h>
#include <spidev.h>

#include "stm_interface/StmInterfaceNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StmInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
