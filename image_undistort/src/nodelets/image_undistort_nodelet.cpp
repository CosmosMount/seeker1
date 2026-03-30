#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// 包含我们已经迁移的 ROS 2 版本的 Impl 头文件
#include "image_undistort/image_undistort.h" 

#include <memory> // 用于 std::shared_ptr

namespace image_undistort {

// 1. 类名可以保留，但继承自 rclcpp::Node
class ImageUndistortNodelet : public rclcpp::Node {
 public:
  // 2. onInit() 替换为构造函数
  explicit ImageUndistortNodelet(const rclcpp::NodeOptions& options)
      : rclcpp::Node("image_undistort_nodelet", options) {
    try {
      // 3. 将 'this' (即 rclcpp::Node* 节点指针) 
      //    传递给我们迁移后的 ImageUndistort Impl 类
      image_undistort_ = std::make_shared<ImageUndistort>(this);
    } catch (const std::runtime_error& e) { // 捕获 Impl 构造函数中可能抛出的异常
      // 4. 使用 ROS 2 日志
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
      // 构造函数失败时，节点不会被加载
    }
  }

 private:
  // 成员变量保持不变
  std::shared_ptr<ImageUndistort> image_undistort_;
};
}  // namespace image_undistort

// 5. 替换 PLUGINLIB 宏
//    使用 rclcpp_components 宏来注册这个类
RCLCPP_COMPONENTS_REGISTER_NODE(image_undistort::ImageUndistortNodelet)