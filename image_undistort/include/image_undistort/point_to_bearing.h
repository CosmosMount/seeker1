#ifndef IMAGE_UNDISTORT_POINT_TO_BEARING_NODELET_H
#define IMAGE_UNDISTORT_POINT_TO_BEARING_NODELET_H

#include <stdio.h>
#include <Eigen/Dense>

// 1. 替换 ROS 1 头文件
#include <rclcpp/rclcpp.hpp>
#include <memory> // 用于 std::shared_ptr

// 2. 替换消息头文件
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <nlopt.h>

#include "image_undistort/undistorter.h" // 假设此文件也被迁移

namespace image_undistort {

// Default values
constexpr int kQueueSize = 10;
constexpr bool kDefaultCameraInfoFromROSParams = false;
const std::string kDefaultCameraNamespace = "camera";

class PointToBearing {
 public:
  // 3. 构造函数签名修改 (NodeHandle -> Node*)
  PointToBearing(rclcpp::Node* node);

  static void optimizeForBearingVector(
      const InputCameraParameters& camera_parameters,
      const Eigen::Vector2d& pixel_location, Eigen::Vector3d* bearing);

  static double bearingProjectionError(unsigned int n, const double* values,
                                       double* grad, void* data);

 private:
  // 4. 回调函数签名修改 (ROS 1 Ptr -> ROS 2 Ptr)
  void imagePointCallback(
      const geometry_msgs::msg::PointStamped::ConstSharedPtr& image_point);

  void cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

  // 5. 替换 NodeHandle
  rclcpp::Node* node_ptr_;

  // 6. 替换 ROS 1 订阅者
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr image_point_sub_;

  // 7. 替换 ROS 1 发布者
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr bearing_pub_;

  // camera info
  std::shared_ptr<InputCameraParameters> camera_parameters_ptr_;

  // other variables
  int queue_size_;
};
}

#endif