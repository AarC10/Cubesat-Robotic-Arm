#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class ImageHandlerNode final : public rclcpp::Node {
public:
  explicit ImageHandlerNode();
  ~ImageHandlerNode() override;

private:
  // Pub Subs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imageRequestSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rawImageSub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressedImagePub;

  std::mutex imageMutex;
  sensor_msgs::msg::Image::SharedPtr lastRawImage;

  // Callbacks
  void handleImageRequest(const std_msgs::msg::Bool::SharedPtr msg);
  void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
  

  // File saving
  std::string saveDirectory;

  // Helpers
  void compress();
  void saveRawImageToDisk();
  
};
