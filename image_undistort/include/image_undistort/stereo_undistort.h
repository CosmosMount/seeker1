#ifndef IMAGE_UNDISTORT_STEREO_UNDISTORTER_H
#define IMAGE_UNDISTORT_STEREO_UNDISTORTER_H

// 1. 替换 ROS 1 头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <memory> // 用于 std::shared_ptr

// 2. 替换消息头文件
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "image_undistort/camera_parameters.h" // 假设此文件也被迁移
#include "image_undistort/undistorter.h"     // 假设此文件也被迁移

namespace image_undistort {

// Default values (保持不变)
constexpr int kQueueSize = 10;
constexpr bool kDefaultInputCameraInfoFromROSParams = true;
const std::string kDefaultFirstCameraNamespace = "first_camera";
const std::string kDefaultSecondCameraNamespace = "second_camera";
constexpr int kDefaultProcessEveryNthFrame = 1;
const std::string kDefaultOutputImageType = "";
constexpr double kDefaultScale = 1.0;
constexpr bool kDefaultPublishTF = true;
const std::string kDefaultFirstOutputFrame = "first_camera_rect";
const std::string kDefaultSecondOutputFrame = "second_camera_rect";
constexpr bool kDefaultRenameInputFrame = false;
const std::string kDefaultFirstInputFrame = "first_camera";
const std::string kDefaultSecondInputFrame = "second_camera";
constexpr bool kDefaultRenameRadtanPlumbBob = false;
constexpr bool kDefaultInvertT = false;

class StereoUndistort {
 public:
  // 3. 构造函数签名修改 (NodeHandle -> Node*)
  StereoUndistort(rclcpp::Node* node);

  // 4. 回调函数签名修改 (ROS 1 Ptr -> ROS 2 Ptr)
  void imagesCallback(const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
                      const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in);

  void camerasCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
      const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info_msg_in,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info_msg_in);

 private:
  int getQueueSize() const;

  void updateUndistorter(const CameraSide& side);

  // 4. 回调函数签名修改 (ROS 1 Ptr -> ROS 2 Ptr)
  void sendCameraInfo(const std_msgs::msg::Header& header, const CameraSide& side,
                      const CameraIO& io);

  void processAndSendImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_in,
                           const CameraSide& side);

  // 5. 替换 TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;

  // 6. 替换 NodeHandle
  rclcpp::Node* node_ptr_;
  std::unique_ptr<image_transport::ImageTransport> it_;

  // subscribers
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>
      first_camera_info_sub_ptr_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>
      second_camera_info_sub_ptr_;

  // 7. 替换 ROS 1 发布者
  image_transport::Publisher first_image_pub_;
  image_transport::Publisher second_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr first_camera_info_input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr second_camera_info_input_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr first_camera_info_output_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr second_camera_info_output_pub_;

  // 8. [消息类型迁移]
  // filters (更新消息类型)
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::Image>
      ImageSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>>
      image_sync_ptr_;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::CameraInfo>
      CameraSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>>
      camera_sync_ptr_;

  // camera parameters
  std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;

  // undistorters
  std::shared_ptr<Undistorter> first_undistorter_ptr_;
  std::shared_ptr<Undistorter> second_undistorter_ptr_;

  // other variables (保持不变)
  bool input_camera_info_from_ros_params_;
  int queue_size_;
  int process_every_nth_frame_;
  std::string output_image_type_;
  bool publish_tf_;
  std::string first_output_frame_;
  std::string second_output_frame_;
  bool rename_input_frame_;
  std::string first_input_frame_;
  std::string second_input_frame_;
  bool rename_radtan_plumb_bob_;
  int frame_counter_;
};
}

#endif
