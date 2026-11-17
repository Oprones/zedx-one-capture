#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class ZedxOneNode : public rclcpp::Node {
public:
  ZedxOneNode() : Node("zedx_one_node") {
    const int device_id = this->declare_parameter<int>("device_id", 0);
    const int width = this->declare_parameter<int>("width", 1920);
    const int height = this->declare_parameter<int>("height", 1200);
    const int fps = this->declare_parameter<int>("fps", 30);
    const bool hdr = this->declare_parameter<bool>("hdr", false);
    jpeg_quality_ = this->declare_parameter<int>("jpeg_quality", 80);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "cam_front_optical_frame");
    const auto image_topic =
      this->declare_parameter<std::string>("image_topic", "cam_front/image_raw/compressed");
    const auto camera_info_topic =
      this->declare_parameter<std::string>("camera_info_topic", "cam_front/camera_info");

    image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      image_topic, rclcpp::SensorDataQoS());
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS());

    oc::ArgusCameraConfig config;
    config.mDeviceId = device_id;
    config.mWidth = width;
    config.mHeight = height;
    config.mFPS = fps;
    config.hdr = hdr;
    config.verbose_level = 1;

    const oc::ARGUS_STATE state = camera_.openCamera(config);
    if (state != oc::ARGUS_STATE::OK) {
      RCLCPP_FATAL(
        this->get_logger(), "Failed to open Argus camera %d (state=%d)",
        device_id, static_cast<int>(state));
      throw std::runtime_error("Failed to open Argus camera");
    }

    cam_width_ = camera_.getWidth();
    cam_height_ = camera_.getHeight();
    cam_channels_ = camera_.getNumberOfChannels();
    raw_frame_ = cv::Mat(cam_height_, cam_width_, CV_8UC(cam_channels_));

    RCLCPP_INFO(
      this->get_logger(), "Opened ZED X One camera %d (%dx%d, %d ch) -> %s",
      device_id, cam_width_, cam_height_, cam_channels_, image_topic.c_str());

    running_.store(true);
    capture_thread_ = std::thread(&ZedxOneNode::captureLoop, this);
  }

  ~ZedxOneNode() override {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    camera_.closeCamera();
  }

private:
  void captureLoop() {
    const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    cv::Mat bgr_frame;

    while (rclcpp::ok() && running_.load()) {
      if (!camera_.isNewFrame()) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        continue;
      }

      const auto stamp = this->now();
      std::memcpy(
        raw_frame_.data, camera_.getPixels(),
        static_cast<size_t>(cam_width_) * cam_height_ * cam_channels_);

      if (cam_channels_ == 4) {
        cv::cvtColor(raw_frame_, bgr_frame, cv::COLOR_BGRA2BGR);
      } else {
        bgr_frame = raw_frame_;
      }

      sensor_msgs::msg::CompressedImage img_msg;
      img_msg.header.stamp = stamp;
      img_msg.header.frame_id = frame_id_;
      img_msg.format = "jpeg";
      cv::imencode(".jpg", bgr_frame, img_msg.data, encode_params);
      image_pub_->publish(img_msg);

      sensor_msgs::msg::CameraInfo info_msg;
      info_msg.header = img_msg.header;
      info_msg.width = static_cast<uint32_t>(cam_width_);
      info_msg.height = static_cast<uint32_t>(cam_height_);
      info_msg.distortion_model = "plumb_bob";
      info_pub_->publish(info_msg);
    }
  }

  oc::ArgusBayerCapture camera_;
  cv::Mat raw_frame_;
  int cam_width_{0};
  int cam_height_{0};
  int cam_channels_{4};
  int jpeg_quality_{80};
  std::string frame_id_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ZedxOneNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("zedx_one_node"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
