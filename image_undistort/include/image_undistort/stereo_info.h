#ifndef IMAGE_UNDISTORT_STEREO_INFO_H
#define IMAGE_UNDISTORT_STEREO_INFO_H

// 1. 替换 ROS 1 头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <memory> // 用于 std::shared_ptr

// 2. 替换消息头文件
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp> // 显式包含 Header

#include "image_undistort/camera_parameters.h" // 假设此文件也被迁移

namespace image_undistort {

// Default values (保持不变)
constexpr int kQueueSize = 10;
constexpr bool kDefaultInputCameraInfoFromROSParams = true;
const std::string kDefaultFirstCameraNamespace = "first_camera";
const std::string kDefaultSecondCameraNamespace = "second_camera";
constexpr double kDefaultScale = 1.0;
constexpr bool kDefaultRenameRadtanPlumbBob = false;
constexpr bool kDefaultInvertT = false;

class StereoInfo {
 public:
  // 3. 构造函数签名修改 (NodeHandle -> Node*)
  StereoInfo(rclcpp::Node* node);

  // 4. 回调函数签名修改 (ROS 1 Ptr -> ROS 2 Ptr)
  void firstImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

  void secondImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

  void firstCameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

  void secondCameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

 private:
  // 4. 回调函数签名修改 (ROS 1 Ptr -> ROS 2 Ptr)
  void sendCameraInfo(const std_msgs::msg::Header& header, const CameraSide& side,
                      const CameraIO& io);

  // 5. 替换 NodeHandle
  rclcpp::Node* node_ptr_;
  std::unique_ptr<image_transport::ImageTransport> it_; // [修改后]

  // subscribers
  image_transport::Subscriber first_image_sub_;
  image_transport::Subscriber second_image_sub_;
  // 6. 替换 ROS 1 订阅者
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr first_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr second_camera_info_sub_;

  // 7. 替换 ROS 1 发布者
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr first_camera_info_input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr second_camera_info_input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr first_camera_info_output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr second_camera_info_output_pub_;

  std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;

  int queue_size_;
  bool rename_radtan_plumb_bob_;
};
}

#endif