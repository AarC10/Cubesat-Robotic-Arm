#include "image_handler/ImageHandlerNode.hpp"

ImageHandlerNode::ImageHandlerNode() : Node("image_compressor") {
  imageRequestSub = this->create_subscription<bool>(
      "image_request", 10,
      std::bind(&ImageHandlerNode::handleImageRequest, this, std::placeholders::_1));

  rawImageSub = this->create_subscription<sensor_msgs::msg::Image>(
      "raw_image", 10,
      std::bind(&ImageHandlerNode::handleRawImage, this, std::placeholders::_1));

  compressedImagePub = this->create_publisher<std_msgs::msg::String>("compressed_image", 10);

  // TODO: Think of better default location on Pi
  saveDirectory = this->declare_parameter<std::string>("save_directory", "/tmp/images");
}

ImageHandlerNode::~ImageHandlerNode() = default;

void ImageHandlerNode::handleImageRequest(const bool sendCompressed) {
  std::lock_guard<std::mutex> lock(imageMutex);
  if (sendCompressed) {
    if (lastRawImage) {
      compress();
    } else {
      RCLCPP_WARN(this->get_logger(), "No raw image available to compress");
    }
  }

  if (lastRawImage) {
    saveRawImageToDisk();
  }

}

void ImageHandlerNode::handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imageMutex);
  lastRawImage = msg;
}

void ImageHandlerNode::saveRawImageToDisk() {
    // TODO: Need to integrate OpenCV
}


void ImageHandlerNode::compress() {
  // TODO: Need to integrate compression lib 
}