#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// 包含 "Depth" 核心逻辑的头文件 (假设它也被迁移了)
#include "image_undistort/depth.h" 

#include <memory> // 用于 std::shared_ptr

namespace image_undistort {

// 1. 类名保留, 继承自 rclcpp::Node
class DepthNodelet : public rclcpp::Node {
 public:
  // 2. onInit() 替换为构造函数
  explicit DepthNodelet(const rclcpp::NodeOptions& options)
      : rclcpp::Node("depth_nodelet", options) { // "depth_nodelet" 是节点名
    try {
      // 3. 将 'this' (rclcpp::Node*) 传递给 Impl 类
      //    (假设 Depth 类的构造函数也已按此方式迁移)
      depth_ = std::make_shared<Depth>(this);
    } catch (const std::runtime_error& e) {
      // 4. 使用 ROS 2 日志
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
    }
  }

 private:
  // 成员变量保持不变
  std::shared_ptr<Depth> depth_;
};
}  // namespace image_undistort

// 5. 替换 PLUGINLIB 宏
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::DepthNodelet)