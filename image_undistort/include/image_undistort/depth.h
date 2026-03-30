#ifndef IMAGE_UNDISTORT_DISPARITY_H
#define IMAGE_UNDISTORT_DISPARITY_H

// ROS 2 和 C++ 头文件
#include <rclcpp/rclcpp.hpp>
#include <memory> 

// ROS 2 库头文件
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp> // <-- 注意：您之前用了 time_synchronizer.hpp，但代码里用的是 Synchronizer

// ROS 2 消息头文件
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace image_undistort {

// ... (默认参数值保持不变) ...
constexpr int kDepthQueueSize = 10;
constexpr double kDelta = 0.000000001;
constexpr int kPreFilterCap = 31;
constexpr int kSADWindowSize = 11;
constexpr int kMinDisparity = 0;
constexpr int kNumDisparities = 64;
constexpr int kUniquenessRatio = 0;
constexpr int kSpeckleRange = 3;
constexpr int kSpeckleWindowSize = 500;
constexpr int kTextureThreshold = 0;
const std::string kPreFilterType = "xsobel";
constexpr int kPreFilterSize = 9;
constexpr bool kUseSGBM = false;
constexpr int kP1 = 120;
constexpr int kP2 = 240;
constexpr int kDisp12MaxDiff = -1;
constexpr bool kUseHHMode = false;
constexpr bool kDoMedianBlur = true;


class Depth {
 public:
  Depth(rclcpp::Node* node);

  void camerasCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
      const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info_msg_in,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info_msg_in);

 private:
  // ... (成员函数声明保持不变) ...
  int getQueueSize() const;
  static bool ApproxEq(double A, double B);
    bool processCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& first_camera_info,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& second_camera_info,
      double* baseline, double* focal_length, bool* first_is_left, int* cx,
      int* cy);
  static void fillDisparityFromSide(const cv::Mat& input_disparity,
                                    const cv::Mat& valid, const bool& from_left,
                                    cv::Mat* filled_disparity);
  void bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                 cv::Mat* disparity_filled,
                                 cv::Mat* input_valid) const;
  void calcDisparityImage(const sensor_msgs::msg::Image::ConstSharedPtr& first_image_msg_in,
                          const sensor_msgs::msg::Image::ConstSharedPtr& second_image_msg_in,
                          cv_bridge::CvImagePtr disparity_ptr) const;
  void calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
                      const double baseline, const double focal_length,
                      const int cx, const int cy,
                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
                      pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud);

  rclcpp::Node* node_ptr_;
  
  // --- 【关键修改 1】 ---
  // 将 it_ 和 camera_sync_ 从对象实例改为智能指针
  std::unique_ptr<image_transport::ImageTransport> it_;

  int queue_size_;

  // subscribers (保持对象实例不变)
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> first_camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> second_camera_info_sub_;

  // publishers
  image_transport::Publisher disparity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_pointcloud_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::CameraInfo>
      CameraSyncPolicy;
      
  // --- 【关键修改 2】 ---
  std::unique_ptr<message_filters::Synchronizer<CameraSyncPolicy>> camera_sync_;

  // ... (参数成员变量保持不变) ...
  int pre_filter_cap_;
  int sad_window_size_;
  int min_disparity_;
  int num_disparities_;
  int uniqueness_ratio_;
  int speckle_range_;
  int speckle_window_size_;
  int texture_threshold_;
  int pre_filter_type_;
  int pre_filter_size_;
  bool use_sgbm_;
  int p1_;
  int p2_;
  int disp_12_max_diff_;
  bool use_mode_HH_;
  bool do_median_blur_;
};
}

#endif