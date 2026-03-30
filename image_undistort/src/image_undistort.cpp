/**
 * @file image_undistort.cpp
 * @brief ROS 2 Migration - Full Source
 */

#include "image_undistort/image_undistort.h"
#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <memory>
#include <functional>

namespace image_undistort {

// 构造函数
ImageUndistort::ImageUndistort(rclcpp::Node* node)
    : node_ptr_(node),
      undistorter_ptr_(nullptr),
      frame_counter_(0) {

  // [关键修正 1] ImageTransport 初始化 (使用空删除器技巧)
  std::shared_ptr<rclcpp::Node> node_shared(node_ptr_, [](rclcpp::Node*){});
  it_ = std::make_unique<image_transport::ImageTransport>(node_shared);

  // 初始化 TF Broadcaster
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_ptr_);

  // [参数声明与获取]
  bool input_camera_info_from_ros_params = 
      node_ptr_->declare_parameter<bool>("input_camera_info_from_ros_params", kDefaultInputCameraInfoFromROSParams);

  rename_radtan_plumb_bob_ = 
      node_ptr_->declare_parameter<bool>("rename_radtan_plumb_bob", kDefaultRenameRadtanPlumbBob);

  std::string output_camera_info_source_in = 
      node_ptr_->declare_parameter<std::string>("output_camera_info_source", kDefaultOutputCameraInfoSource);

  if (output_camera_info_source_in == "auto_generated") {
    output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
  } else if (output_camera_info_source_in == "match_input") {
    output_camera_info_source_ = OutputInfoSource::MATCH_INPUT;
  } else if (output_camera_info_source_in == "ros_params") {
    output_camera_info_source_ = OutputInfoSource::ROS_PARAMS;
  } else if (output_camera_info_source_in == "camera_info") {
    output_camera_info_source_ = OutputInfoSource::CAMERA_INFO;
  } else {
    RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Invalid camera source given, valid options are auto_generated, "
        "match_input, ros_params and camera_info. Defaulting to "
        "auto_generated");
    output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
  }

  queue_size_ = node_ptr_->declare_parameter<int>("queue_size", kImageQueueSize);
  if (queue_size_ < 1) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  process_image_ = node_ptr_->declare_parameter<bool>("process_image", kDefaultProcessImage);
  if (!process_image_ && !input_camera_info_from_ros_params) {
    RCLCPP_FATAL(
        node_ptr_->get_logger(),
        "Settings specify no image processing and not to generate camera info "
        "from file. This leaves nothing for the node to do, exiting");
    throw std::runtime_error("Invalid configuration");
  }

  scale_ = node_ptr_->declare_parameter<double>("scale", kDefaultScale);

  bool undistort_image = 
      node_ptr_->declare_parameter<bool>("undistort_image", kDefaultUndistortImage);
  
  DistortionProcessing distortion_processing;
  if (undistort_image) {
    distortion_processing = DistortionProcessing::UNDISTORT;
  } else {
    distortion_processing = DistortionProcessing::PRESERVE;
  }
  camera_parameters_pair_ptr_ =
      std::make_shared<CameraParametersPair>(distortion_processing);

  process_every_nth_frame_ = 
      node_ptr_->declare_parameter<int>("process_every_nth_frame", kDefaultProcessEveryNthFrame);
  
  output_image_type_ = 
      node_ptr_->declare_parameter<std::string>("output_image_type", kDefaultOutputImageType);
  
  // check output type string is correctly formatted
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
  output_frame_ = node_ptr_->declare_parameter<std::string>("output_frame", kDefaultOutputFrame);
  if (output_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Output frame cannot be blank, setting to default");
    output_frame_ = kDefaultOutputFrame;
  }

  rename_input_frame_ = 
      node_ptr_->declare_parameter<bool>("rename_input_frame", kDefaultRenameInputFrame);
  
  input_frame_ = 
      node_ptr_->declare_parameter<std::string>("input_frame", kDefaultInputFrame);
  if (input_frame_.empty()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Input frame cannot be blank, setting to default");
    input_frame_ = kDefaultInputFrame;
  }

  // [订阅者初始化]
  std::string input_camera_namespace;
  if (input_camera_info_from_ros_params) {
    input_camera_namespace = 
        node_ptr_->declare_parameter<std::string>("input_camera_namespace", kDefaultInputCameraNamespace);
    
    if (!camera_parameters_pair_ptr_->setCameraParameters(
            node_ptr_, input_camera_namespace, CameraIO::INPUT)) {
      RCLCPP_FATAL(node_ptr_->get_logger(), "Loading of input camera parameters failed, exiting");
      throw std::runtime_error("Loading input parameters failed");
    }
    
    // [修正 2] 使用 std::bind 绑定回调
    image_sub_ = it_->subscribe("input/image", queue_size_,
                                std::bind(&ImageUndistort::imageCallback, this, std::placeholders::_1));
  } else {
    camera_sub_ = it_->subscribeCamera("input/image", queue_size_,
                                        std::bind(&ImageUndistort::cameraCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // [发布者初始化]
  if (process_image_) {
    bool pub_camera_info_output = true;
    if (output_camera_info_source_ == OutputInfoSource::ROS_PARAMS) {
      std::string output_camera_namespace = 
          node_ptr_->declare_parameter<std::string>("output_camera_namespace", kDefaultOutputCameraNamespace);
      
      if (!camera_parameters_pair_ptr_->setCameraParameters(
              node_ptr_, output_camera_namespace, CameraIO::OUTPUT)) {
        RCLCPP_FATAL(node_ptr_->get_logger(), "Loading of output camera parameters failed, exiting");
        throw std::runtime_error("Loading output parameters failed");
      }
    } else if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
      camera_parameters_pair_ptr_->setOutputFromInput(scale_);
    } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
      camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
    } else {
      // [修正 3] 使用 Lambda 包装回调，避免 bind 类型推导错误
      camera_info_sub_ =
          node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
              "output/camera_info", queue_size_,
              [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                  this->cameraInfoCallback(msg);
              });
      pub_camera_info_output = false;
    }

    if (pub_camera_info_output) {
      camera_pub_ = it_->advertiseCamera("output/image", queue_size_);
    } else {
      image_pub_ = it_->advertise("output/image", queue_size_);
    }
  } else {
    camera_parameters_pair_ptr_->setOutputFromInput(scale_);
    camera_info_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "output/camera_info", queue_size_);
  }
}

void ImageUndistort::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  if (!process_image_) {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = image_msg_in->header;
    if (rename_input_frame_) {
      camera_info.header.frame_id = input_frame_;
    }
    camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                           &camera_info);
    if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
      camera_info.distortion_model = "plumb_bob";
    }
    camera_info_pub_->publish(camera_info);
    return;
  }
  
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);

  std::string encoding = image_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and using it will crash anything that does a
    // color conversion
    encoding = "mono8";
  }
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, encoding));

  // if undistorter not built or built using old data update it
  if (!undistorter_ptr_ || (undistorter_ptr_->getCameraParametersPair() !=
                            *camera_parameters_pair_ptr_)) {
    try {
      undistorter_ptr_ =
          std::make_shared<Undistorter>(*camera_parameters_pair_ptr_);
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
      return;
    }
  }

  undistorter_ptr_->undistortImage(image_in_ptr->image,
                                   &(image_out_ptr->image));

  image_out_ptr->header.frame_id = output_frame_;

  if (publish_tf_) {
    Eigen::Matrix4d T =
        camera_parameters_pair_ptr_->getInputPtr()->T().inverse() *
        camera_parameters_pair_ptr_->getOutputPtr()->T();

    // [关键修正 4] TF 转换
    // 使用 Isometry3d 来匹配 tf2::eigenToTransform 的签名
    Eigen::Isometry3d T_iso(T);

    geometry_msgs::msg::TransformStamped t_stamped;
    // 只赋值 transform 部分
    t_stamped.transform = tf2::eigenToTransform(T_iso).transform;
    t_stamped.header.stamp = image_out_ptr->header.stamp;
    
    std::string frame = image_in_ptr->header.frame_id;
    if (rename_input_frame_) {
      frame = input_frame_;
    }
    if (frame.empty()) {
      RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Image frame name is blank, cannot construct tf");
    } else {
      t_stamped.header.frame_id = frame;
      t_stamped.child_frame_id = output_frame_;
      br_->sendTransform(t_stamped);
    }
  }

  // if camera info was just read in from a topic don't republish it
  if (output_camera_info_source_ == OutputInfoSource::CAMERA_INFO) {
    image_pub_.publish(*(image_out_ptr->toImageMsg()));
  } else {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = image_out_ptr->header;
    if (rename_input_frame_) {
      camera_info.header.frame_id = input_frame_;
    }
    camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                           &camera_info);
    if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
      camera_info.distortion_model = "plumb_bob";
    }
    camera_pub_.publish(*(image_out_ptr->toImageMsg()), camera_info);
  }
}

void ImageUndistort::cameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  
  camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                 CameraIO::INPUT);
  if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
    camera_parameters_pair_ptr_->setOutputFromInput(scale_);
  } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
    camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
  }

  imageCallback(image_msg);
}

void ImageUndistort::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  
  if (!camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                      CameraIO::OUTPUT)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Setting output camera from ros message failed");
  }
}
} // namespace image_undistort