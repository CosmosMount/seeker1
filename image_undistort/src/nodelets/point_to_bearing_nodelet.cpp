#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// 包含 "PointToBearing" 核心逻辑的头文件 (假设它也将被迁移)
#include "image_undistort/point_to_bearing.h" 

#include <memory> // 用于 std::shared_ptr

namespace image_undistort {

// 1. 类名保留, 继承自 rclcpp::Node
class PointToBearingNodelet : public rclcpp::Node {
 public:
  // 2. onInit() 替换为构造函数
  explicit PointToBearingNodelet(const rclcpp::NodeOptions& options)
      : rclcpp::Node("point_to_bearing_nodelet", options) {
    try {
      // 3. 将 'this' (rclcpp::Node*) 传递给 Impl 类
      //    (假设 PointToBearing 类的构造函数也已按此方式迁移)
      point_to_bearing_ = std::make_shared<PointToBearing>(this);
    } catch (const std::runtime_error& e) {
      // 4. 使用 ROS 2 日志
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
    }
  }

 private:
  // 成员变量保持不变
  std::shared_ptr<PointToBearing> point_to_bearing_;
};
}  // namespace image_undistort

// 5. 替换 PLUGINLIB 宏
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::PointToBearingNodelet)