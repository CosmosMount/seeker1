#ifndef IMAGE_UNDISTORT_IMAGE_CONCAT_H
#define IMAGE_UNDISTORT_IMAGE_CONCAT_H

// 包含所有用到的类型的头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

namespace image_undistort {

// 定义同步策略
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, 
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

// 类的声明
class ImageConcat {
public:
    // 构造函数的声明
    explicit ImageConcat(rclcpp::Node* node);

private:
    // 成员函数的声明
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg0,
        const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
        const sensor_msgs::msg::Image::ConstSharedPtr& msg2,
        const sensor_msgs::msg::Image::ConstSharedPtr& msg3);

    cv::Mat processImages(const std::vector<cv::Mat>& images, int method);

    // 成员变量的声明
    rclcpp::Node* node_;
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::SubscriberFilter sub_0_, sub_1_, sub_2_, sub_3_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    image_transport::Publisher pub_;
    
    int queue_size_;
    int concat_method_;
    int concat_width_;
    int concat_height_;
    double blend_weight_;
    double pano_radius_;
    double bev_ground_height_;
    double bev_fx_;
    int process_every_nth_frame_;
    int frame_count_{0};
};

} // namespace image_undistort

#endif // IMAGE_UNDISTORT_IMAGE_CONCAT_H