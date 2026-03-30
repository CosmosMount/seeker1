/**
 * @file stereo_undistort.cpp
 * @brief ROS 2 Migration complete.
 */

#include "image_undistort/stereo_undistort.h"
#include "image_undistort/camera_parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

namespace image_undistort {

// 1. 构造函数
StereoUndistort::StereoUndistort(rclcpp::Node* node)
    : node_ptr_(node),
      first_undistorter_ptr_(nullptr),
      second_undistorter_ptr_(nullptr),
      frame_counter_(0) {
  
  // [关键修正 1] 创建一个不管理的 shared_ptr (空删除器) 传给需要 shared_ptr 的库
  std::shared_ptr<rclcpp::Node> node_shared(node_ptr_, [](rclcpp::Node*){});
  
  // 初始化 ImageTransport (使用 unique_ptr 管理)
  it_ = std::make_unique<image_transport::ImageTransport>(node_shared);
  
  // 初始化 TF Broadcaster
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_ptr_);

  // [参数声明]
  queue_size_ = node_ptr_->declare_parameter<int>("queue_size", kQueueSize);
  if (queue_size_ < 1) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  input_camera_info_from_ros_params_ =
      node_ptr_->declare_parameter<bool>("input_camera_info_from_ros_params",
                                        kDefaultInputCameraInfoFromROSParams);

  rename_radtan_plumb_bob_ = 
      node_ptr_->declare_parameter<bool>("rename_radtan_plumb_bob",
                                       kDefaultRenameRadtanPlumbBob);

  bool invert_T = node_ptr_->declare_parameter<bool>("invert_T", kDefaultInvertT);
  double scale = node_ptr_->declare_parameter<double>("scale", kDefaultScale);

  stereo_camera_parameters_ptr_ =
      std::make_shared<StereoCameraParameters>(scale);

  process_every_nth_frame_ = 
      node_ptr_->declare_parameter<int>("process_every_nth_frame",
                                     kDefaultProcessEveryNthFrame);
  output_image_type_ = 
      node_ptr_->declare_parameter<std::string>("output_image_type",
                                             kDefaultOutputImageType);
  
  if (!output_image_type_.empty()) {
    try {
      cv_bridge::getCvType(output_image_type_);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR_STREAM(
          node_ptr_->get_logger(),
          "cv_bridge error while setting output_image_type, output will match "
          "input type. "
          << e.what());
      output_image_type_ = "";
    }
  }

  publish_tf_ = node_ptr_->declare_parameter<bool>("publish_tf", kDefaultPublishTF);
  first_output_frame_ = 
      node_ptr_->declare_parameter<std::string>("first_output_frame",
                                              kDefaultFirstOutputFrame);
  if (first_output_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "First output frame cannot be blank, setting to default");
    first_output_frame_ = kDefaultFirstOutputFrame;
  }
  second_output_frame_ = 
      node_ptr_->declare_parameter<std::string>("second_output_frame",
                                               kDefaultSecondOutputFrame);
  if (second_output_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Second output frame cannot be blank, setting to default");
    second_output_frame_ = kDefaultSecondOutputFrame;
  }

  rename_input_frame_ = 
      node_ptr_->declare_parameter<bool>("rename_input_frame",
                                       kDefaultRenameInputFrame);
  first_input_frame_ = 
      node_ptr_->declare_parameter<std::string>("first_input_frame",
                                              kDefaultFirstInputFrame);
  if (first_input_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "First input frame cannot be blank, setting to default");
    first_input_frame_ = kDefaultFirstInputFrame;
  }
  second_input_frame_ = 
      node_ptr_->declare_parameter<std::string>("second_input_frame",
                                               kDefaultSecondInputFrame);
  if (second_input_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Second input frame cannot be blank, setting to default");
    second_input_frame_ = kDefaultSecondInputFrame;
  }

  // [发布者初始化]
  first_camera_info_output_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "rect/first/camera_info", queue_size_);
  second_camera_info_output_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "rect/second/camera_info", queue_size_);
  first_image_pub_ = it_->advertise("rect/first/image", queue_size_);
  second_image_pub_ = it_->advertise("rect/second/image", queue_size_);

  // [关键修正 2] 订阅者初始化
  // image_transport::SubscriberFilter 在 ROS 2 中第一个参数需要 Node*
  first_image_sub_.subscribe(node_ptr_, "raw/first/image", "raw", rmw_qos_profile_sensor_data);
  second_image_sub_.subscribe(node_ptr_, "raw/second/image", "raw", rmw_qos_profile_sensor_data);

  if (input_camera_info_from_ros_params_) {
    std::string first_camera_namespace = 
        node_ptr_->declare_parameter<std::string>("first_camera_namespace",
                                                kDefaultFirstCameraNamespace);
    std::string second_camera_namespace = 
        node_ptr_->declare_parameter<std::string>("second_camera_namespace",
                                                kDefaultSecondCameraNamespace);
    
    if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
            node_ptr_, first_camera_namespace, CameraSide::FIRST, invert_T) ||
        !stereo_camera_parameters_ptr_->setInputCameraParameters(
            node_ptr_, second_camera_namespace, CameraSide::SECOND,
            invert_T)) {
      RCLCPP_FATAL(node_ptr_->get_logger(), "Loading of input camera parameters failed, exiting");
      throw std::runtime_error("Loading of input camera parameters failed");
    }

    first_camera_info_input_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "raw/first/camera_info", queue_size_);
    second_camera_info_input_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "raw/second/camera_info", queue_size_);

    image_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
            ImageSyncPolicy(queue_size_), first_image_sub_, second_image_sub_);
    
    image_sync_ptr_->registerCallback(
        std::bind(&StereoUndistort::imagesCallback, this, std::placeholders::_1, std::placeholders::_2));

  } else {
    // [关键修正 3] message_filters 使用 node_shared (shared_ptr)
    first_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
            node_shared, "raw/first/camera_info", rmw_qos_profile_sensor_data);
    second_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
            node_shared, "raw/second/camera_info", rmw_qos_profile_sensor_data);

    camera_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(
            CameraSyncPolicy(queue_size_), first_image_sub_, second_image_sub_,
            *first_camera_info_sub_ptr_, *second_camera_info_sub_ptr_);
    
    camera_sync_ptr_->registerCallback(
        std::bind(&StereoUndistort::camerasCallback, this, std::placeholders::_1, 
                  std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
}

void StereoUndistort::updateUndistorter(const CameraSide& side) {
  std::shared_ptr<Undistorter>* undistorter_ptr_ptr;
  CameraParametersPair camera_parameters_pair;

  if (side == CameraSide::FIRST) {
    undistorter_ptr_ptr = &first_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getFirst();
  } else {
    undistorter_ptr_ptr = &second_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getSecond();
  }

  if (!(*undistorter_ptr_ptr) ||
      ((*undistorter_ptr_ptr)->getCameraParametersPair() !=
       camera_parameters_pair)) {
    try {
      *undistorter_ptr_ptr =
          std::make_shared<Undistorter>(camera_parameters_pair);
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
      return;
    }
  }
}

void StereoUndistort::sendCameraInfo(const std_msgs::msg::Header& header,
                                     const CameraSide& side,
                                     const CameraIO& io) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header = header;

  try {
    stereo_camera_parameters_ptr_->generateCameraInfoMessage(side, io,
                                                             &camera_info);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
    return;
  }

  if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
    camera_info.distortion_model = "plumb_bob";
  }

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

void StereoUndistort::processAndSendImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_in, const CameraSide& side) {
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);

  std::string encoding = image_in_ptr->encoding;
  if (encoding == "8UC1") {
    encoding = "mono8";
  }
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, encoding));

  updateUndistorter(side);

  if (side == CameraSide::FIRST) {
    image_out_ptr->header.frame_id = first_output_frame_;

    first_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                           &(image_out_ptr->image));
    // 直接解引用发布
    first_image_pub_.publish(*(image_out_ptr->toImageMsg()));

    if (publish_tf_) {
      Eigen::Matrix4d T =
          stereo_camera_parameters_ptr_->getFirst().getOutputPtr()->T();

      geometry_msgs::msg::TransformStamped t_stamped;
      t_stamped.header.stamp = image_out_ptr->header.stamp;

      std::string frame = image_in_ptr->header.frame_id;
      if (rename_input_frame_) {
        frame = first_input_frame_;
      }
      if (frame.empty()) {
        RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Image frame name is blank, cannot construct tf");
      } else {
        t_stamped.header.frame_id = frame;
        t_stamped.child_frame_id = first_output_frame_;
        
        // [关键修正 4] TF 转换修正
        Eigen::Isometry3d T_iso(T);
        t_stamped.transform = tf2::eigenToTransform(T_iso).transform;
        
        br_->sendTransform(t_stamped);
      }
    }
  } else {
    image_out_ptr->header.frame_id = second_output_frame_;

    second_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                            &(image_out_ptr->image));
    // 直接解引用发布
    second_image_pub_.publish(*(image_out_ptr->toImageMsg()));

    if (publish_tf_) {
      Eigen::Matrix4d T =
          stereo_camera_parameters_ptr_->getSecond().getOutputPtr()->T();
      
      geometry_msgs::msg::TransformStamped t_stamped;
      t_stamped.header.stamp = image_out_ptr->header.stamp;

      std::string frame = image_in_ptr->header.frame_id;
      if (rename_input_frame_) {
        frame = second_input_frame_;
      }
      if (frame.empty()) {
        RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Image frame name is blank, cannot construct tf");
      } else {
        t_stamped.header.frame_id = frame;
        t_stamped.child_frame_id = second_output_frame_;
        
        // [关键修正 4] TF 转换修正
        Eigen::Isometry3d T_iso(T);
        t_stamped.transform = tf2::eigenToTransform(T_iso).transform;
        
        br_->sendTransform(t_stamped);
      }
    }
  }
}

void StereoUndistort::imagesCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
    const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in) {
  if (!stereo_camera_parameters_ptr_->valid()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Camera parameters invalid, undistortion failed");
    return;
  }

  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  processAndSendImage(first_image_msg_in, CameraSide::FIRST);
  processAndSendImage(second_image_msg_in, CameraSide::SECOND);

  if (input_camera_info_from_ros_params_) {
    std_msgs::msg::Header header = first_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = first_input_frame_;
    }
    sendCameraInfo(header, CameraSide::FIRST, CameraIO::INPUT);

    header = second_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = second_input_frame_;
    }
    sendCameraInfo(header, CameraSide::SECOND, CameraIO::INPUT);
  }

  std_msgs::msg::Header header = first_image_msg_in->header;
  header.frame_id = first_output_frame_;
  sendCameraInfo(header, CameraSide::FIRST, CameraIO::OUTPUT);

  header = second_image_msg_in->header;
  header.frame_id = second_output_frame_;
  sendCameraInfo(header, CameraSide::SECOND, CameraIO::OUTPUT);
}

void StereoUndistort::camerasCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
    const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info_msg_in,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info_msg_in) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *first_camera_info_msg_in, CameraSide::FIRST) ||
      !stereo_camera_parameters_ptr_->setInputCameraParameters(
          *second_camera_info_msg_in, CameraSide::SECOND)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Setting camera info failed, dropping frame");
    return;
  }

  imagesCallback(first_image_msg_in, second_image_msg_in);
}
}  // namespace image_undistort