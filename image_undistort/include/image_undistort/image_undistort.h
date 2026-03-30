#ifndef IMAGE_UNDISTORT_IMAGE_UNDISTORT_NODELET_H
#define IMAGE_UNDISTORT_IMAGE_UNDISTORT_NODELET_H

#include <stdio.h>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_undistort/undistorter.h"

namespace image_undistort {

// Default values (保持不变)
constexpr int kImageQueueSize = 10;
constexpr bool kDefaultInputCameraInfoFromROSParams = false;
const std::string kDefaultOutputCameraInfoSource = "auto_generated";
const std::string kDefaultInputCameraNamespace = "input_camera";
const std::string kDefaultOutputCameraNamespace = "output_camera";
constexpr bool kDefaultProcessImage = true;
constexpr bool kDefaultUndistortImage = true;
constexpr int kDefaultProcessEveryNthFrame = 1;
const std::string kDefaultOutputImageType = "";
constexpr double kDefaultScale = 1.0;
constexpr bool kDefaultPublishTF = true;
const std::string kDefaultOutputFrame = "output_camera";
constexpr bool kDefaultRenameInputFrame = false;
const std::string kDefaultInputFrame = "input_camera";
constexpr bool kDefaultRenameRadtanPlumbBob = false;

class ImageUndistort {
 public:
  ImageUndistort(rclcpp::Node* node);

 private:
  // [修正] 使用 ROS 2 类型 ConstSharedPtr
  void imageMsgToCvMat(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       cv::Mat* image);

  void updateCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info);

  bool loadCameraParameters(const CameraIO& io,
                            sensor_msgs::msg::CameraInfo* loaded_camera_info,
                            std::string* image_topic);

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_in);

  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

  rclcpp::Node* node_ptr_;
  std::unique_ptr<image_transport::ImageTransport> it_; // 确保是 unique_ptr

  // subscribers
  image_transport::Subscriber image_sub_;
  image_transport::CameraSubscriber camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // publishers
  image_transport::CameraPublisher camera_pub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  std::shared_ptr<Undistorter> undistorter_ptr_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<CameraParametersPair> camera_parameters_pair_ptr_;

  enum class OutputInfoSource {
    AUTO_GENERATED,
    MATCH_INPUT,
    ROS_PARAMS,
    CAMERA_INFO
  };

  int queue_size_;
  bool process_image_;
  OutputInfoSource output_camera_info_source_;
  int process_every_nth_frame_;
  std::string output_image_type_;
  double scale_;
  bool publish_tf_;
  std::string output_frame_;
  bool rename_input_frame_;
  std::string input_frame_;
  bool rename_radtan_plumb_bob_;

  int frame_counter_;
};
}

#endif