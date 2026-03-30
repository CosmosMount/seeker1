#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "image_undistort/image_concat.h"

namespace image_undistort {

class ImageConcatNodelet : public rclcpp::Node {
public:
    explicit ImageConcatNodelet(const rclcpp::NodeOptions& options)
        : rclcpp::Node("image_concat_nodelet", options) {
        try {
            impl_ = std::make_shared<ImageConcat>(this);
        } catch (const std::runtime_error& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to init ImageConcat: %s", e.what());
        }
    }

private:
    std::shared_ptr<ImageConcat> impl_;
};

} // namespace image_undistort

// 注册组件 (这使得它可以被 launch 文件加载)
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::ImageConcatNodelet)