/**
 * @file stereo_info.cpp
 * @brief ROS 2 Migration complete.
 */

#include "image_undistort/stereo_info.h"
#include "image_undistort/camera_parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory> // for std::shared_ptr, std::make_unique

namespace image_undistort {

// 1. 构造函数签名修改
StereoInfo::StereoInfo(rclcpp::Node* node)
    : node_ptr_(node) { // 移除初始化列表中的 it_
  
  // [关键修正 1] 创建一个不管理的 shared_ptr (空删除器) 传给需要 shared_ptr 的库
  std::shared_ptr<rclcpp::Node> node_shared(node_ptr_, [](rclcpp::Node*){});
  
  // 初始化 ImageTransport (使用 unique_ptr 管理)
  // 注意：请确保头文件 stereo_info.h 中 it_ 被声明为 std::unique_ptr<image_transport::ImageTransport>
  it_ = std::make_unique<image_transport::ImageTransport>(node_shared);

  // [参数声明]
  bool input_camera_info_from_ros_params = 
      node_ptr_->declare_parameter<bool>("input_camera_info_from_ros_params",
                                        kDefaultInputCameraInfoFromROSParams);

  queue_size_ = node_ptr_->declare_parameter<int>("queue_size", kQueueSize);
  if (queue_size_ < 1) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  rename_radtan_plumb_bob_ = 
      node_ptr_->declare_parameter<bool>("rename_radtan_plumb_bob",
                                       kDefaultRenameRadtanPlumbBob);

  bool invert_T = node_ptr_->declare_parameter<bool>("invert_T", kDefaultInvertT);
  double scale = node_ptr_->declare_parameter<double>("scale", kDefaultScale);

  stereo_camera_parameters_ptr_ =
      std::make_shared<StereoCameraParameters>(scale);

  // [订阅者与发布者初始化]
  if (input_camera_info_from_ros_params) {
    std::string first_camera_namespace = 
        node_ptr_->declare_parameter<std::string>("first_camera_namespace",
                                                kDefaultFirstCameraNamespace);
    std::string second_camera_namespace = 
        node_ptr_->declare_parameter<std::string>("second_camera_namespace",
                                                kDefaultSecondCameraNamespace);
    
    // 假设 setInputCameraParameters 已迁移
    if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
            node_ptr_, first_camera_namespace, CameraSide::FIRST, invert_T) ||
        !stereo_camera_parameters_ptr_->setInputCameraParameters(
            node_ptr_, second_camera_namespace, CameraSide::SECOND,
            invert_T)) {
      RCLCPP_FATAL(node_ptr_->get_logger(), "Loading of input camera parameters failed, exiting");
      throw std::runtime_error("Loading of input camera parameters failed");
    }
    
    // [关键修正 2] 使用 it_->subscribe (因为是 unique_ptr)
    // 注意：image_transport::ImageTransport::subscribe 返回的是 Subscriber，不是 SubscriberFilter
    // 这里我们直接绑定回调
    first_image_sub_ = it_->subscribe("raw/first/image", queue_size_,
                                      std::bind(&StereoInfo::firstImageCallback, this, std::placeholders::_1));
    second_image_sub_ = it_->subscribe("raw/second/image", queue_size_,
                                       std::bind(&StereoInfo::secondImageCallback, this, std::placeholders::_1));

    first_camera_info_input_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "raw/first/camera_info", queue_size_);
    second_camera_info_input_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "raw/second/camera_info", queue_size_);

  } else {
    // 替换 nh_.subscribe
    first_camera_info_sub_ =
        node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "raw/first/camera_info", queue_size_,
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                this->firstCameraInfoCallback(msg);
            });
            
    second_camera_info_sub_ =
        node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
            "raw/second/camera_info", queue_size_,
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                this->secondCameraInfoCallback(msg);
            });
  }

  // 替换 nh_.advertise
  first_camera_info_output_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "rect/first/camera_info", queue_size_);
  second_camera_info_output_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "rect/second/camera_info", queue_size_);
}

// [回调签名迁移]
void StereoInfo::firstImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::FIRST, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::FIRST, CameraIO::OUTPUT);
}

void StereoInfo::secondImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::SECOND, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::SECOND, CameraIO::OUTPUT);
}

void StereoInfo::firstCameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::FIRST)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Setting first camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::FIRST, CameraIO::OUTPUT);
  }
}

void StereoInfo::secondCameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::SECOND)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Setting second camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::SECOND, CameraIO::OUTPUT);
  }
}

// [回调签名/类型迁移]
void StereoInfo::sendCameraInfo(const std_msgs::msg::Header& header,
                                const CameraSide& side, const CameraIO& io) {
  sensor_msgs::msg::CameraInfo camera_info; // [消息类型迁移]
  camera_info.header = header;
  try {
    stereo_camera_parameters_ptr_->generateCameraInfoMessage(side, io,
                                                             &camera_info);
  } catch (const std::runtime_error& e) { // 捕获 const 引用
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
    return;
  }

  if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
    camera_info.distortion_model = "plumb_bob";
  }

  // [发布迁移]
  if (side == CameraSide::FIRST) {
    if (io == CameraIO::INPUT) {
      first_camera_info_input_pub_->publish(camera_info);
    } else {
      first_camera_info_output_pub_->publish(camera_info);
    }
  } else {
    if (io == CameraIO::INPUT) {
      second_camera_info_input_pub_->publish(camera_info);
    } else {
      second_camera_info_output_pub_->publish(camera_info);
    }
  }
}

} // namespace image_undistort