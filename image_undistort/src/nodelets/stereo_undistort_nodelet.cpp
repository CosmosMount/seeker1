#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// 包含 "StereoUndistort" 核心逻辑的头文件 (假设它也将被迁移)
#include "image_undistort/stereo_undistort.h" 

#include <memory> // 用于 std::shared_ptr

namespace image_undistort {

// 1. 类名保留, 继承自 rclcpp::Node
class StereoUndistortNodelet : public rclcpp::Node {
 public:
  // 2. onInit() 替换为构造函数
  explicit StereoUndistortNodelet(const rclcpp::NodeOptions& options)
      : rclcpp::Node("stereo_undistort_nodelet", options) {
    try {
      // 3. 将 'this' (rclcpp::Node*) 传递给 Impl 类
      //    (假设 StereoUndistort 类的构造函数也已按此方式迁移)
      stereo_undistort_ = std::make_shared<StereoUndistort>(this);
    } catch (const std::runtime_error& e) {
      // 4. 使用 ROS 2 日志
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
    }
  }

 private:
  // 成员变量保持不变
  std::shared_ptr<StereoUndistort> stereo_undistort_;
};
}  // namespace image_undistort

// 5. 替换 PLUGINLIB 宏
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::StereoUndistortNodelet)