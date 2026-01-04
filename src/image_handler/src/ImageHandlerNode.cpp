#include "image_handler/ImageHandlerNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <sstream>

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
  if (!lastRawImage) {
    RCLCPP_WARN(this->get_logger(), "No raw image to save");
    return;
  }

  try {
    std::filesystem::create_directories(saveDirectory);

    const auto cvImage = cv_bridge::toCvShare(lastRawImage, lastRawImage->encoding);

    std::ostringstream pathBuilder;
    pathBuilder << saveDirectory;
    if (!saveDirectory.empty() && saveDirectory.back() != '/') {
      pathBuilder << '/';
    }
    pathBuilder << "image_" << this->now().nanoseconds() << ".png";

    const auto filePath = pathBuilder.str();
    if (cv::imwrite(filePath, cvImage->image)) {
      RCLCPP_INFO(this->get_logger(), "Saved image to %s", filePath.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to write image to %s", filePath.c_str());
    }
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception while saving image: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while saving image: %s", e.what());
  }
}


void ImageHandlerNode::compress() {
  // TODO: Need to integrate compression lib 
}