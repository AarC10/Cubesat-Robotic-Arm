#include "stm_interface/StmInterfaceNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StmInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
