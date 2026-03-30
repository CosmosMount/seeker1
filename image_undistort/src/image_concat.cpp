#include "image_undistort/image_concat.h" // 必须包含对应的头文件
#include <functional> // for std::bind, std::placeholders

// 其他的 include 已经在 .h 文件中了，这里不需要重复

namespace image_undistort {

// 构造函数的实现
ImageConcat::ImageConcat(rclcpp::Node* node) 
    : node_(node) {
    
    std::shared_ptr<rclcpp::Node> node_shared(node_, [](rclcpp::Node*){});
    it_ = std::make_unique<image_transport::ImageTransport>(node_shared);

    queue_size_ = node_->declare_parameter<int>("queue_size", 10);
    concat_method_ = node_->declare_parameter<int>("concat_method", 3);
    concat_width_ = node_->declare_parameter<int>("concat_width", 1280);
    concat_height_ = node_->declare_parameter<int>("concat_height", 640);
    blend_weight_ = node_->declare_parameter<double>("blend_weight", 0.4);
    pano_radius_ = node_->declare_parameter<double>("pano_radius", 300.0);
    bev_ground_height_ = node_->declare_parameter<double>("bev_ground_height", -1.5);
    bev_fx_ = node_->declare_parameter<double>("bev_fx", 100.0);
    process_every_nth_frame_ = node_->declare_parameter<int>("process_every_nth_frame", 1);
    
    rclcpp::QoS qos(queue_size_);
    auto rmw_qos = qos.get_rmw_qos_profile();

    sub_0_.subscribe(node_, "raw/first/image", "raw", rmw_qos);
    sub_1_.subscribe(node_, "raw/second/image", "raw", rmw_qos);
    sub_2_.subscribe(node_, "raw/third/image", "raw", rmw_qos);
    sub_3_.subscribe(node_, "raw/fourth/image", "raw", rmw_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size_), sub_0_, sub_1_, sub_2_, sub_3_
    );
    
    sync_->registerCallback(std::bind(&ImageConcat::imageCallback, this, 
        std::placeholders::_1, std::placeholders::_2, 
        std::placeholders::_3, std::placeholders::_4));

    pub_ = it_->advertise("rect/image", 1);
    
    RCLCPP_INFO(node_->get_logger(), "ImageConcat initialized. Method: %d, Output: %dx%d", 
        concat_method_, concat_width_, concat_height_);
}

// imageCallback 函数的实现
void ImageConcat::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg0,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg2,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg3) 
{
    if (++frame_count_ < process_every_nth_frame_) return;
    frame_count_ = 0;

    try {
        std::vector<cv::Mat> images(4);
        images[0] = cv_bridge::toCvShare(msg0, "bgr8")->image;
        images[1] = cv_bridge::toCvShare(msg1, "bgr8")->image;
        images[2] = cv_bridge::toCvShare(msg2, "bgr8")->image;
        images[3] = cv_bridge::toCvShare(msg3, "bgr8")->image;

        cv::Mat result = processImages(images, concat_method_);

        if (!result.empty()) {
            std_msgs::msg::Header header = msg0->header;
            auto out_msg = cv_bridge::CvImage(header, "bgr8", result).toImageMsg();
            pub_.publish(*out_msg);
        }

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

// processImages 函数的实现
cv::Mat ImageConcat::processImages(const std::vector<cv::Mat>& images, int method) {
    cv::Mat final_image;
    if (method >= 0) {
        for(const auto& img : images) {
            if(img.empty()) return cv::Mat();
        }
        cv::Mat top, bottom;
        cv::Mat img0, img1, img2, img3;
        cv::Size sz = images[0].size();
        
        cv::resize(images[0], img0, sz);
        cv::resize(images[1], img1, sz);
        cv::resize(images[2], img2, sz);
        cv::resize(images[3], img3, sz);

        cv::hconcat(img0, img1, top);
        cv::hconcat(img3, img2, bottom);
        cv::vconcat(top, bottom, final_image);
        
        if (concat_width_ > 0 && concat_height_ > 0) {
            cv::resize(final_image, final_image, cv::Size(concat_width_, concat_height_));
        }
    } 
    return final_image;
}

} // namespace image_undistort