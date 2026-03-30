#include <memory>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <image_geometry/stereo_camera_model.h> // .h for Humble

#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace stereo_image_proc {

using namespace sensor_msgs::msg;
using namespace stereo_msgs::msg;
using namespace message_filters::sync_policies;

class DepthImageNodelet : public rclcpp::Node
{
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  
  typedef ExactTime<Image, CameraInfo, CameraInfo, DisparityImage> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image_;

  // Processing state
  image_geometry::StereoCameraModel model_;
  cv::Mat_<cv::Vec3f> points_mat_;

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& l_image_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& l_info_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& r_info_msg,
               const stereo_msgs::msg::DisparityImage::ConstSharedPtr& disp_msg);

public:
  explicit DepthImageNodelet(const rclcpp::NodeOptions & options)
    : Node("depth_image_nodelet", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing DepthImageNodelet...");

    int queue_size = this->declare_parameter<int>("queue_size", 5);
    bool approx = this->declare_parameter<bool>("approximate_sync", false);
    
    // [修正 1] 直接订阅，不使用 connectCb 懒加载
    rclcpp::QoS qos(1);
    auto rmw_qos = qos.get_rmw_qos_profile();

    // [修正 2] 修复 subscribe 参数
    // 第1个参数传 this (Node*), 第3个参数传 "raw" (transport)
    sub_l_image_.subscribe(this, "left/image_rect_color", "raw", rmw_qos);
    
    sub_l_info_.subscribe(this, "left/camera_info", rmw_qos);
    sub_r_info_.subscribe(this, "right/camera_info", rmw_qos);
    sub_disparity_.subscribe(this, "disparity", rmw_qos);

    if (approx)
    {
      approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                    sub_l_image_, sub_l_info_,
                                                    sub_r_info_, sub_disparity_) );
      approximate_sync_->registerCallback(
        std::bind(&DepthImageNodelet::imageCb, this,
                  std::placeholders::_1, std::placeholders::_2, 
                  std::placeholders::_3, std::placeholders::_4));
    }
    else
    {
      exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                        sub_l_image_, sub_l_info_,
                                        sub_r_info_, sub_disparity_) );
      exact_sync_->registerCallback(
        std::bind(&DepthImageNodelet::imageCb, this,
                  std::placeholders::_1, std::placeholders::_2, 
                  std::placeholders::_3, std::placeholders::_4));
    }

    // 创建发布者
    pub_depth_image_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", qos);
  }
};

inline bool isValidPoint(const cv::Vec3f& pt)
{
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void DepthImageNodelet::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& l_image_msg,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& l_info_msg,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& r_info_msg,
                                  const stereo_msgs::msg::DisparityImage::ConstSharedPtr& disp_msg)
{
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  const Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;

  auto depth_image = std::make_unique<sensor_msgs::msg::Image>();

  depth_image->header = disp_msg->header;
  depth_image->width = dimage.width;
  depth_image->height = dimage.height;
  depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_image->step = sizeof(float) * depth_image->width;

  size_t num_pixels = depth_image->width * depth_image->height;

  depth_image->data.resize(num_pixels * sizeof(float));

  std::vector<uint8_t>& data = depth_image->data;

  float nan = std::numeric_limits<float>::quiet_NaN();

  for (size_t i = 0; i < num_pixels; ++i){
    if (mat(i)[2] < 1000.0){
      memcpy (&data[i*sizeof(float)], &mat(i)[2], sizeof (float));
    } else {
      memcpy (&data[i*sizeof(float)], &nan, sizeof (float));
    }
  }

  pub_depth_image_->publish(std::move(depth_image));
}

} // namespace stereo_image_proc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_image_proc::DepthImageNodelet)