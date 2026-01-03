#include "status_accumulator/StatusAccumulatorNode.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusAccumulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
