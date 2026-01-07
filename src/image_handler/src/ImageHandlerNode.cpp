#include "image_handler/ImageHandlerNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <sstream>
#include <thread>
#include <algorithm>

ImageHandlerNode::ImageHandlerNode() : Node("image_compressor") {
  imageRequestSub = this->create_subscription<arm_msgs::msg::ImageRequest>(
      "image_request", 10,
      std::bind(&ImageHandlerNode::handleImageRequest, this, std::placeholders::_1));

  rawImageSub = this->create_subscription<sensor_msgs::msg::Image>(
      "raw_image", 10,
      std::bind(&ImageHandlerNode::handleRawImage, this, std::placeholders::_1));

  compressedImagePub = this->create_publisher<std_msgs::msg::String>("compressed_image", 10);
  imageDataPub = this->create_publisher<arm_msgs::msg::ImageData>("image_data", 10);

  // TODO: Think of better default location on Pi
  saveDirectory = this->declare_parameter<std::string>("save_directory", "/tmp/images");
}

ImageHandlerNode::~ImageHandlerNode() = default;

void ImageHandlerNode::handleImageRequest(const arm_msgs::msg::ImageRequest::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imageMutex);
  
  if (!lastRawImage) {
    RCLCPP_WARN(this->get_logger(), "No raw image available for request ID %u", msg->image_id);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Processing image request ID %u (compress=%d, quality=%u)", 
              msg->image_id, msg->compress, msg->quality);
  
  saveRawImageToDisk();
  compressAndTransmit(msg->image_id, msg->compress, msg->quality);
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

void ImageHandlerNode::compressAndTransmit(uint8_t imageId, bool compress, uint8_t quality) {
  if (!lastRawImage) {
    RCLCPP_WARN(this->get_logger(), "No raw image to compress and transmit");
    return;
  }

  try {
    const auto cvImage = cv_bridge::toCvShare(lastRawImage, lastRawImage->encoding);
    
    // TODO: compression

    // For now, encode as JPEG
    std::vector<uint8_t> encodedImage;
    std::vector<int> compressionParams;
    compressionParams.push_back(cv::IMWRITE_JPEG_QUALITY);
    compressionParams.push_back(95);
    
    if (!cv::imencode(".jpg", cvImage->image, encodedImage, compressionParams)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode image as JPEG");
      return;
    }

    const uint32_t totalSize = static_cast<uint32_t>(encodedImage.size());
    const uint32_t totalChunks = (totalSize + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;

    RCLCPP_INFO(this->get_logger(), "Image %u encoded: %u bytes, %u chunks", 
                imageId, totalSize, totalChunks);

    // Publish image data in chunks
    for (uint32_t chunkIndex = 0; chunkIndex < totalChunks; ++chunkIndex) {
      const size_t offset = chunkIndex * MAX_CHUNK_SIZE;
      const size_t chunkSize = std::min(MAX_CHUNK_SIZE, totalSize - offset);

      arm_msgs::msg::ImageData imageData;
      imageData.image_id = imageId;
      imageData.total_chunks = totalChunks;
      imageData.chunk_index = chunkIndex;
      imageData.total_size = totalSize;
      imageData.data.assign(encodedImage.begin() + offset, 
                           encodedImage.begin() + offset + chunkSize);

      imageDataPub->publish(imageData);
      
      RCLCPP_DEBUG(this->get_logger(), "Published chunk %u/%u (%zu bytes)", 
                   chunkIndex + 1, totalChunks, chunkSize);
      
      // Small delay between chunks to avoid overwhelming the radio
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(this->get_logger(), "Completed transmission of image %u", imageId);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during image encoding: %s", e.what());
  }
}