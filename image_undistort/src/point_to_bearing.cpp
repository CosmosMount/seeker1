#include "image_undistort/point_to_bearing.h"
#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"
#include <rclcpp/rclcpp.hpp> // 确保 rclcpp API 可用
#include <geometry_msgs/msg/point_stamped.hpp> // 包含消息定义

namespace image_undistort {

// 1. 构造函数签名修改 (NodeHandle -> Node*)
PointToBearing::PointToBearing(rclcpp::Node* node)
    : node_ptr_(node) { // 2. 存储 Node*
  
  // 3. [参数迁移] 替换 nh_private_.param
  bool camera_info_from_ros_params = 
      node_ptr_->declare_parameter<bool>("camera_info_from_ros_params",
                                        kDefaultCameraInfoFromROSParams);

  queue_size_ = node_ptr_->declare_parameter<int>("queue_size", kQueueSize);
  if (queue_size_ < 1) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  // 4. [参数/构造函数迁移]
  std::string camera_namespace;
  if (camera_info_from_ros_params) {
    camera_namespace = 
        node_ptr_->declare_parameter<std::string>("camera_namespace",
                                                kDefaultCameraNamespace);

    try {
      // 假设 InputCameraParameters 构造函数也已迁移
      camera_parameters_ptr_ = std::make_shared<InputCameraParameters>(
          node_ptr_, camera_namespace);
    } catch (const std::runtime_error& e) { // 捕获 const 引用
      RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
      // 5. [错误处理迁移] 替换 ROS_FATAL/exit
      RCLCPP_FATAL(node_ptr_->get_logger(), "Loading of input camera parameters failed, exiting");
      throw; // 重新抛出异常以使构造失败
    }

  } else {
    // 6. [订阅者迁移] 替换 nh_.subscribe
    camera_info_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", queue_size_,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            this->cameraInfoCallback(msg);
        });
  }

  image_point_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PointStamped>(
      "image_point", queue_size_,
      [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          this->imagePointCallback(msg);
      });

  // 7. [发布者迁移] 替换 nh_.advertise
  bearing_pub_ =
      node_ptr_->create_publisher<geometry_msgs::msg::PointStamped>("bearing", queue_size_);
}

// 8. [回调签名迁移]
void PointToBearing::imagePointCallback(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr& image_point) {
  if (camera_parameters_ptr_ == nullptr) {
    // 9. [日志迁移]
    RCLCPP_WARN(node_ptr_->get_logger(), "No camera calibration, cannot calculate bearing");
    return;
  }

  const Eigen::Vector2d pixel_location(image_point->point.x,
                                       image_point->point.y);

  Eigen::Vector3d bearing;
  optimizeForBearingVector(*camera_parameters_ptr_, pixel_location, &bearing);

  geometry_msgs::msg::PointStamped bearing_msg;
  bearing_msg.header = image_point->header;
  bearing_msg.point.x = bearing.x();
  bearing_msg.point.y = bearing.y();
  bearing_msg.point.z = bearing.z();
  
  // 10. [发布迁移] 使用 ->
  bearing_pub_->publish(bearing_msg);
}

// ...
// NLopt/Eigen 纯算术逻辑 (Lines 80-136) 无需修改
// ...
// 替换这个函数
void PointToBearing::optimizeForBearingVector(
    const InputCameraParameters& camera_parameters,
    const Eigen::Vector2d& pixel_location, Eigen::Vector3d* bearing) {
  const Eigen::Vector2d inital_guess =
      (camera_parameters.P().topLeftCorner<3, 3>().inverse() *
       Eigen::Vector3d(pixel_location[0], pixel_location[1], 1.0))
          .head<2>();

  std::pair<const Eigen::Vector2d&, const InputCameraParameters&> data_pair =
      std::make_pair(pixel_location, camera_parameters);

  // --- 修改开始: 从 C++ API 改为 C API ---

  // 1. 创建 nlopt 对象。C++: nlopt::opt opt(...) -> C: nlopt_create(...)
  nlopt_opt opt = nlopt_create(NLOPT_LN_NELDERMEAD, 2);

  // 2. 设置目标函数。C++: opt.set_min_objective(...) -> C: nlopt_set_min_objective(opt, ...)
  nlopt_set_min_objective(opt, PointToBearing::bearingProjectionError,
                          static_cast<void*>(&data_pair));
  
  // 3. 设置停止条件。C++: opt.set_xtol_rel(...) -> C: nlopt_set_xtol_rel(opt, ...)
  nlopt_set_xtol_rel(opt, 1e-3);

  std::vector<double> values = {inital_guess.x(), inital_guess.y()};
  double minf;

  // 4. 执行优化。C++: opt.optimize(...) -> C: nlopt_optimize(...)
  //    注意：C API 需要一个指向 double 数组的指针，所以我们用 &values[0]
  nlopt_result result = nlopt_optimize(opt, values.data(), &minf);

  // 5. 【重要】释放 nlopt 对象。C API 需要手动释放内存，否则会内存泄漏。
  nlopt_destroy(opt);

  // --- 修改结束 ---

  *bearing = Eigen::Vector3d(values[0], values[1], 1.0);
  bearing->normalize();
}

// 在 point_to_bearing.cpp 文件中

// ... (其他代码)

// 【用下面的完整函数替换你原来的 bearingProjectionError 函数】
double PointToBearing::bearingProjectionError(unsigned int n, const double* values,
                                              double* grad, void* data) {
  // (可选，但推荐) 避免"未使用参数"的警告
  (void)n;
  (void)grad;

  // split out inputs
  // 现在 values 是一个裸指针，但我们仍然可以像数组一样访问它
  const Eigen::Vector2d pixel_location(values[0], values[1]);
  const std::pair<const Eigen::Vector2d&, const InputCameraParameters&>
      data_pair = *static_cast<
          std::pair<const Eigen::Vector2d&, const InputCameraParameters&>*>(
          data);
  const Eigen::Vector2d& distorted_pixel_location = data_pair.first;
  const InputCameraParameters& camera_parameters = data_pair.second;

  Eigen::Matrix<double, 3, 4> P_bearing;
  P_bearing.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  P_bearing.topRightCorner<3, 1>() = Eigen::Vector3d::Zero();

  Eigen::Vector2d estimated_distorted_pixel_location;
  Undistorter::distortPixel(camera_parameters.K(), camera_parameters.R(),
                            P_bearing, camera_parameters.distortionModel(),
                            camera_parameters.D(), pixel_location,
                            &estimated_distorted_pixel_location);

  return (estimated_distorted_pixel_location - distorted_pixel_location).norm();
}

// ... (其他代码)

// 8. [回调签名迁移]
void PointToBearing::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
  try {
    // 假设 InputCameraParameters 构造函数也已迁移
    camera_parameters_ptr_ =
        std::make_shared<InputCameraParameters>(*camera_info);
  } catch (const std::runtime_error& e) { // 捕获 const 引用
    RCLCPP_ERROR(node_ptr_->get_logger(), "%s", e.what());
  }
}
} // namespace image_undistort