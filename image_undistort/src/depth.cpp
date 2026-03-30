#include "image_undistort/depth.h"
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace image_undistort {

Depth::Depth(rclcpp::Node* node)
    : node_ptr_(node)
{
    it_ = std::make_unique<image_transport::ImageTransport>(
        std::shared_ptr<rclcpp::Node>(node_ptr_, [](rclcpp::Node*){})
    );
    
    queue_size_ = node_ptr_->declare_parameter<int>("queue_size", kDepthQueueSize);
    rclcpp::QoS qos(queue_size_);
    auto rmw_qos = qos.get_rmw_qos_profile();

    // --- 【最终修正：将 it_.get() 改为 node_ptr_】 ---
    first_image_sub_.subscribe(node_ptr_, "rect/first/image", "raw", rmw_qos);
    second_image_sub_.subscribe(node_ptr_, "rect/second/image", "raw", rmw_qos);
    
    first_camera_info_sub_.subscribe(node_ptr_, "rect/first/camera_info", rmw_qos);
    second_camera_info_sub_.subscribe(node_ptr_, "rect/second/camera_info", rmw_qos);
    
    camera_sync_ = std::make_unique<message_filters::Synchronizer<CameraSyncPolicy>>(
        CameraSyncPolicy(queue_size_), first_image_sub_,
        second_image_sub_, first_camera_info_sub_,
        second_camera_info_sub_);

    // 【修正 std::bind 的占位符】
    camera_sync_->registerCallback(
        std::bind(&Depth::camerasCallback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3,
                  std::placeholders::_4));

    std::string pre_filter_type_string = 
        node_ptr_->declare_parameter<std::string>("pre_filter_type", kPreFilterType);
    if (pre_filter_type_string == std::string("xsobel")) {
        pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
    } else if (pre_filter_type_string == std::string("normalized_response")) {
        pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
    } else {
        throw std::runtime_error(
            "Unrecognized prefilter type, choices are 'xsobel' or "
            "'normalized_response'");
    }
    min_disparity_ = node_ptr_->declare_parameter<int>("min_disparity", kMinDisparity);
    num_disparities_ = node_ptr_->declare_parameter<int>("num_disparities", kNumDisparities);
    pre_filter_cap_ = node_ptr_->declare_parameter<int>("pre_filter_cap", kPreFilterCap);
    uniqueness_ratio_ = node_ptr_->declare_parameter<int>("uniqueness_ratio", kUniquenessRatio);
    speckle_range_ = node_ptr_->declare_parameter<int>("speckle_range", kSpeckleRange);
    speckle_window_size_ = node_ptr_->declare_parameter<int>("speckle_window_size",
                                                  kSpeckleWindowSize);
    sad_window_size_ = node_ptr_->declare_parameter<int>("sad_window_size", kSADWindowSize);
    texture_threshold_ = node_ptr_->declare_parameter<int>("texture_threshold", kTextureThreshold);
    pre_filter_size_ = node_ptr_->declare_parameter<int>("pre_filter_size", kPreFilterSize);
    use_sgbm_ = node_ptr_->declare_parameter<bool>("use_sgbm", kUseSGBM);
    p1_ = node_ptr_->declare_parameter<int>("p1", kP1);
    p2_ = node_ptr_->declare_parameter<int>("p2", kP2);
    disp_12_max_diff_ = node_ptr_->declare_parameter<int>("disp_12_max_diff", kDisp12MaxDiff);
    use_mode_HH_ = node_ptr_->declare_parameter<bool>("use_mode_HH", kUseHHMode);
    do_median_blur_ = node_ptr_->declare_parameter<bool>("do_median_blur", kDoMedianBlur);

    disparity_pub_ = it_->advertise("disparity/image", queue_size_);
    pointcloud_pub_ =
        node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", queue_size_);
    freespace_pointcloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "freespace_pointcloud", queue_size_);
}

// 您的所有其他成员函数实现都保持不变 (我将它们折叠以保持简洁)
#if 1
void Depth::fillDisparityFromSide(const cv::Mat& input_disparity,
                                  const cv::Mat& valid, const bool& from_left,
                                  cv::Mat* filled_disparity) {
  *filled_disparity =
      cv::Mat(input_disparity.rows, input_disparity.cols, CV_16S);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    bool prev_valid = false;
    int16_t prev_value;

    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      size_t x_scan;
      if (from_left) {
        x_scan = x_pixels;
      } else {
        x_scan = (input_disparity.cols - x_pixels - 1);
      }

      if (valid.at<uint8_t>(y_pixels, x_scan)) {
        prev_valid = true;
        prev_value = input_disparity.at<int16_t>(y_pixels, x_scan);
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
            std::numeric_limits<int16_t>::max();
      } else if (prev_valid) {
        filled_disparity->at<int16_t>(y_pixels, x_scan) = prev_value;
      } else {
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
            std::numeric_limits<int16_t>::max();
      }
    }
  }
}
void Depth::bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                        cv::Mat* disparity_filled,
                                        cv::Mat* input_valid) const {
  *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  int side_bound = sad_window_size_ / 2;

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      if ((x_pixels < side_bound + min_disparity_ + num_disparities_) ||
          (y_pixels < side_bound) ||
          (x_pixels > (input_disparity.cols - side_bound)) ||
          (y_pixels > (input_disparity.rows - side_bound)) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) < 0) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) >=
           (min_disparity_ + num_disparities_ - 1) * 16)) {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(sad_window_size_, sad_window_size_));
  cv::erode(*input_valid, *input_valid, kernel);

  cv::Mat disparity_filled_left, disparity_filled_right;
  fillDisparityFromSide(input_disparity, *input_valid, true,
                        &disparity_filled_left);
  fillDisparityFromSide(input_disparity, *input_valid, false,
                        &disparity_filled_right);

  *disparity_filled = cv::max(disparity_filled_left, disparity_filled_right);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      if (input_disparity.at<int16_t>(y_pixels, x_pixels) == 0) {
        disparity_filled->at<int16_t>(y_pixels, x_pixels) = 1;
      }
    }
  }
}

void Depth::calcPointCloud(
    const cv::Mat& input_disparity, const cv::Mat& left_image,
    const double baseline, const double focal_length, const int cx,
    const int cy, pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
    pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud) {
  pointcloud->clear();
  freespace_pointcloud->clear();

  if (left_image.depth() != CV_8U) {
    RCLCPP_ERROR(node_ptr_->get_logger(),
        "Pointcloud generation is currently only supported on 8 bit images");
    return;
  }

  cv::Mat disparity_filled, input_valid;
  bulidFilledDisparityImage(input_disparity, &disparity_filled, &input_valid);

  int side_bound = sad_window_size_ / 2;
  for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
       ++y_pixels) {
    for (int x_pixels = side_bound + min_disparity_ + num_disparities_;
         x_pixels < input_disparity.cols - side_bound;
         ++x_pixels) {
      const uint8_t& is_valid = input_valid.at<uint8_t>(y_pixels, x_pixels);
      const int16_t& input_value =
          input_disparity.at<int16_t>(y_pixels, x_pixels);
      const int16_t& filled_value =
          disparity_filled.at<int16_t>(y_pixels, x_pixels);

      bool freespace;
      double disparity_value;

      if (filled_value < std::numeric_limits<int16_t>::max()) {
        disparity_value = static_cast<double>(filled_value);
        freespace = true;
      }
      else if (is_valid) {
        disparity_value = static_cast<double>(input_value);
        freespace = false;
      } else {
        continue;
      }

      pcl::PointXYZRGB point;

      point.z = (16 * focal_length * baseline) / disparity_value;
      point.x = point.z * (x_pixels - cx) / focal_length;
      point.y = point.z * (y_pixels - cy) / focal_length;

      if (left_image.channels() == 3) {
        const cv::Vec3b& color = left_image.at<cv::Vec3b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else if (left_image.channels() == 4) {
        const cv::Vec4b& color = left_image.at<cv::Vec4b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else {
        point.b = left_image.at<uint8_t>(y_pixels, x_pixels);
        point.g = point.b;
        point.r = point.b;
      }

      if (freespace) {
        freespace_pointcloud->push_back(point);
      } else {
        pointcloud->push_back(point);
      }
    }
  }
}
void Depth::calcDisparityImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_image_msg,
    cv_bridge::CvImagePtr disparity_ptr) const {
  cv_bridge::CvImageConstPtr left_ptr =
      cv_bridge::toCvShare(left_image_msg, "mono8");

  cv_bridge::CvImageConstPtr right_ptr =
      cv_bridge::toCvShare(right_image_msg, "mono8");

#if (defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH == 2)
#else
  if (use_sgbm_) {
    int mode;
    if (use_mode_HH_) {
      mode = cv::StereoSGBM::MODE_HH;
    } else {
      mode = cv::StereoSGBM::MODE_SGBM;
    }

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        min_disparity_, num_disparities_, sad_window_size_, p1_, p2_,
        disp_12_max_diff_, pre_filter_cap_, uniqueness_ratio_,
        speckle_window_size_, speckle_range_, mode);

    left_matcher->compute(left_ptr->image, right_ptr->image,
                          disparity_ptr->image);

  } else {
    cv::Ptr<cv::StereoBM> left_matcher =
        cv::StereoBM::create(num_disparities_, sad_window_size_);

    left_matcher->setPreFilterType(pre_filter_type_);
    left_matcher->setPreFilterSize(pre_filter_size_);
    left_matcher->setPreFilterCap(pre_filter_cap_);
    left_matcher->setMinDisparity(min_disparity_);
    left_matcher->setTextureThreshold(texture_threshold_);
    left_matcher->setUniquenessRatio(uniqueness_ratio_);
    left_matcher->setSpeckleRange(speckle_range_);
    left_matcher->setSpeckleWindowSize(speckle_window_size_);
    left_matcher->compute(left_ptr->image, right_ptr->image,
                          disparity_ptr->image);
  }
#endif

  if(do_median_blur_){
    cv::medianBlur(disparity_ptr->image, disparity_ptr->image, 5);
  }
}

bool Depth::processCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info, double* baseline,
    double* focal_length, bool* first_is_left, int* cx, int* cy) {
  if (first_camera_info->height != second_camera_info->height) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Image heights do not match");
    return false;
  }
  if (first_camera_info->width != second_camera_info->width) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Image widths do not match");
    return false;
  }

  for (double d : first_camera_info->d) {
    if (!ApproxEq(d, 0)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "First image has non-zero distortion");
      return false;
    }
  }
  for (double d : second_camera_info->d) {
    if (!ApproxEq(d, 0)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Second image has non-zero distortion");
      return false;
    }
  }

  for (size_t i = 0; i < 12; ++i) {
    if ((i != 3) &&
        !ApproxEq(first_camera_info->p[i], second_camera_info->p[i])) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Image P matrices must match (excluding x offset)");
      return false;
    }
  }

  if (!ApproxEq(first_camera_info->p[1], 0) ||
      !ApproxEq(first_camera_info->p[4], 0)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Image P matrix contains skew");
    return false;
  }

  if (!ApproxEq(first_camera_info->p[0], first_camera_info->p[5])) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Image P matrix has different values for Fx and Fy");
    return false;
  }

  if (first_camera_info->p[0] <= 0) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Focal length must be greater than 0");
    return false;
  }

  if (!ApproxEq(first_camera_info->p[8], 0) ||
      !ApproxEq(first_camera_info->p[9], 0) ||
      !ApproxEq(first_camera_info->p[10], 1) ||
      !ApproxEq(first_camera_info->p[11], 0)) {
    RCLCPP_WARN_ONCE(
        node_ptr_->get_logger(),
        "Image P matrix does not end in [0,0,1,0], these values will be "
        "ignored");
  }

  if (!ApproxEq(first_camera_info->p[7], 0)) {
    RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "P contains Y offset, this value will be ignored");
  }

  *focal_length = first_camera_info->p[0];
  *baseline = (second_camera_info->p[3] - first_camera_info->p[3]) /
              first_camera_info->p[0];
  if (*baseline > 0) {
    *first_is_left = false;
  } else {
    *first_is_left = true;
    *baseline *= -1;
  }
  *cx = first_camera_info->p[2];
  *cy = first_camera_info->p[6];

  return true;
}

bool Depth::ApproxEq(double A, double B) { return (std::abs(A - B) <= kDelta); }

void Depth::camerasCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info) {
  double baseline, focal_length;
  bool first_is_left;
  int cx, cy;

  if (!processCameraInfo(first_camera_info, second_camera_info, &baseline,
                         &focal_length, &first_is_left, &cx, &cy)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Camera info processing failed, skipping disparity generation");
    return;
  }

  sensor_msgs::msg::Image::ConstSharedPtr left_image_msg;
  sensor_msgs::msg::Image::ConstSharedPtr right_image_msg;

  if (first_is_left) {
    left_image_msg = first_image_msg;
    right_image_msg = second_image_msg;
  } else {
    left_image_msg = second_image_msg;
    right_image_msg = first_image_msg;
  }

  cv::Mat disparity_image =
      cv::Mat(left_image_msg->height, left_image_msg->width, CV_16S);
  cv_bridge::CvImagePtr disparity_ptr(new cv_bridge::CvImage(
      left_image_msg->header, "mono16", disparity_image));

  calcDisparityImage(left_image_msg, right_image_msg, disparity_ptr);
  disparity_pub_.publish(*(disparity_ptr->toImageMsg()));

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> freespace_pointcloud;
  cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_image_msg);
  calcPointCloud(disparity_ptr->image, left_ptr->image, baseline, focal_length,
                 cx, cy, &pointcloud, &freespace_pointcloud);
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header = left_image_msg->header;
  pointcloud_pub_->publish(pointcloud_msg);

  sensor_msgs::msg::PointCloud2 freespace_pointcloud_msg;
  pcl::toROSMsg(freespace_pointcloud, freespace_pointcloud_msg);
  freespace_pointcloud_msg.header = left_image_msg->header;
  freespace_pointcloud_pub_->publish(freespace_pointcloud_msg);
}

int Depth::getQueueSize() const {
  return queue_size_;
}
#endif

} // namespace image_undistort