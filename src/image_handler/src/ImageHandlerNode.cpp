#include "image_handler/ImageCompressorNode.hpp"

ImageCompressorNode::ImageCompressorNode() : Node("image_compressor") {
  imageRequestSub = this->create_subscription<bool>(
      "image_request", 10,
      std::bind(&ImageCompressorNode::handleImageRequest, this, std::placeholders::_1));

  rawImageSub = this->create_subscription<sensor_msgs::msg::Image>(
      "raw_image", 10,
      std::bind(&ImageCompressorNode::handleRawImage, this, std::placeholders::_1));

  compressedImagePub = this->create_publisher<std_msgs::msg::String>("compressed_image", 10);

  // TODO: Think of better default location on Pi
  saveDirectory = this->declare_parameter<std::string>("save_directory", "/tmp/images");
}

void ImageCompressorNode::handleImageRequest(const bool sendCompressed) {
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

void ImageCompressorNode::handleRawImage(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imageMutex);
  lastRawImage = msg;
}

void ImageCompressorNode::saveRawImageToDisk() {
    // TODO: Need to integrate OpenCV
}


void ImageCompressorNode::compress() {
  // TODO: Need to integrate compression lib 
}