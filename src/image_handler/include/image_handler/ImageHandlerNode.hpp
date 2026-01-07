#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "arm_msgs/msg/image_request.hpp"
#include "arm_msgs/msg/image_data.hpp"

class ImageHandlerNode final : public rclcpp::Node {
public:
  explicit ImageHandlerNode();
  ~ImageHandlerNode() override;

private:
  // Pub Subs
  rclcpp::Subscription<arm_msgs::msg::ImageRequest>::SharedPtr imageRequestSub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rawImageSub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressedImagePub;
  rclcpp::Publisher<arm_msgs::msg::ImageData>::SharedPtr imageDataPub;

  std::mutex imageMutex;
  sensor_msgs::msg::Image::SharedPtr lastRawImage;

  // Callbacks
  void handleImageRequest(const arm_msgs::msg::ImageRequest::SharedPtr msg);
  void handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg);
  

  // File saving
  std::string saveDirectory;

  // Image transmission
  static constexpr size_t MAX_CHUNK_SIZE = 200; // Max bytes per chunk for radio transmission

  // Helpers
  void compress();
  void compressAndTransmit(uint8_t imageId, bool compress, uint8_t quality);
  void saveRawImageToDisk();
  
};
