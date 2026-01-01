#include "stm_interface/StmInterfaceNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StmInterfaceNode>("/dev/spidev0.0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
