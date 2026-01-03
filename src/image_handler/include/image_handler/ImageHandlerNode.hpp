#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mutex>

class ImageHandlerNode final : public rclcpp::Node {
public:
  explicit ImageHandlerNode();
  ~ImageHandlerNode() override;

private:
  // Pub Subs
  rclcpp::Subscription<bool>::SharedPtr imageRequestSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rawImageSub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressedImagePub;

  std::mutex imageMutex;
  sensor_msgs::msg::Image::SharedPtr lastRawImage;

  // Callbacks
  void handleImageRequest(const bool sendCompressed);
  void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
  

  // File saving
  std::string saveDirectory;

  // Helpers
  void compress();
  void saveRawImageToDisk();
  
};
