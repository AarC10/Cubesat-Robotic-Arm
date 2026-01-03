#include "image_handler/ImageCompressorNode.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCompressorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
