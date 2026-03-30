#ifndef IMAGE_UNDISTORT_CAMERA_PARAMETERS_H
#define IMAGE_UNDISTORT_CAMERA_PARAMETERS_H

// 1. 替换 ROS 1 头文件
#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/camera_info.hpp> // 2. 替换消息头文件

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

namespace image_undistort {

// Enums (保持不变)
enum class CameraSide { FIRST, SECOND };
enum class CameraIO { INPUT, OUTPUT };
enum class DistortionModel {
  NONE,
  RADTAN,
  EQUIDISTANT,
  FOV,
  OMNI,
  OMNIRADTAN,
  DOUBLESPHERE,
  UNIFIED,
  EXTENDEDUNIFIED
};
enum class DistortionProcessing { UNDISTORT, PRESERVE };

// holds basic properties of a camera
class BaseCameraParameters {
 public:
  // 3. 构造函数签名修改 (NodeHandle -> Node*)
  BaseCameraParameters(rclcpp::Node* node,
                       const std::string& camera_namespace,
                       const bool invert_T);

  // 4. 消息类型修改
  BaseCameraParameters(const sensor_msgs::msg::CameraInfo& camera_info);

  BaseCameraParameters(const cv::Size& resolution_in,
                       const Eigen::Matrix<double, 4, 4>& T_in,
                       const Eigen::Matrix<double, 3, 3>& K_in);

  const cv::Size& resolution() const;  // get image size

  const Eigen::Matrix<double, 4, 4>& T() const;  // get transformation matrix
  const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> R()
      const;  // get rotation matrix
  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> p()
      const;  // get position vector

  const Eigen::Matrix<double, 3, 4>& P() const;  // get projection matrix
  const Eigen::Matrix<double, 3, 3>& K() const;  // get camera matrix

  bool operator==(const BaseCameraParameters& B) const;
  bool operator!=(const BaseCameraParameters& B) const;

 private:
  // 5. [删除]
  //    XmlRpcToMatrix 函数在 ROS 2 中不再需要，
  //    参数加载将在 .cpp 中使用 node->get_parameter() 实现。
  /*
  template <typename Derived>
  static void xmlRpcToMatrix(const XmlRpc::XmlRpcValue& const_input,
                             Eigen::MatrixBase<Derived>* output) { ... }
  */

  cv::Size resolution_;
  Eigen::Matrix<double, 4, 4> T_;
  Eigen::Matrix<double, 3, 4> P_;
  Eigen::Matrix<double, 3, 3> K_;
};

// basic camera properties + distortion parameters
class InputCameraParameters : public BaseCameraParameters {
 public:
  // 3. 构造函数签名修改 (NodeHandle -> Node*)
  InputCameraParameters(rclcpp::Node* node,
                        const std::string& camera_namespace,
                        const bool invert_T = false);

  // 4. 消息类型修改
  InputCameraParameters(const sensor_msgs::msg::CameraInfo& camera_info);

  InputCameraParameters(const cv::Size& resolution_in,
                        const Eigen::Matrix<double, 4, 4>& T_in,
                        const Eigen::Matrix<double, 3, 3>& K_in,
                        const std::vector<double>& D_in,
                        const DistortionModel& distortion_model);

  const std::vector<double>& D() const;  // get distortion vector
  const DistortionModel& distortionModel() const;

  bool operator==(const InputCameraParameters& B) const;
  bool operator!=(const InputCameraParameters& B) const;

 private:
  static const DistortionModel stringToDistortion(
      const std::string& distortion_model, const std::string& camera_model);

  std::vector<double> D_;
  DistortionModel distortion_model_;
};

// basic camera properties + anything special to output cameras (currently
// nothing)
class OutputCameraParameters : public BaseCameraParameters {
 public:
  using BaseCameraParameters::BaseCameraParameters;
};

// holds the camera parameters of the input camera and virtual output camera
class CameraParametersPair {
 public:
  CameraParametersPair(const DistortionProcessing distortion_processing =
                           DistortionProcessing::UNDISTORT);

  // 3. 方法签名修改 (NodeHandle -> Node*)
  bool setCameraParameters(rclcpp::Node* node,
                           const std::string& camera_namespace,
                           const CameraIO& io, const bool invert_T = false);

  // 4. 消息类型修改
  bool setCameraParameters(const sensor_msgs::msg::CameraInfo& camera_info,
                           const CameraIO& io);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 3>& K,
                                const std::vector<double>& D,
                                const DistortionModel& distortion_model);

  bool setOutputCameraParameters(const cv::Size& resolution,
                                 const Eigen::Matrix<double, 4, 4>& T,
                                 const Eigen::Matrix<double, 3, 3>& K);

  bool setOutputFromInput(const double scale);

  bool setOptimalOutputCameraParameters(const double scale);

  const DistortionProcessing& distortionProcessing() const;

  // 4. 消息类型修改
  void generateCameraInfoMessage(const CameraIO& io,
                                 sensor_msgs::msg::CameraInfo* camera_info) const;

  const std::shared_ptr<InputCameraParameters>& getInputPtr() const;
  const std::shared_ptr<OutputCameraParameters>& getOutputPtr() const;

  bool valid() const;
  bool valid(const CameraIO& io) const;

  bool operator==(const CameraParametersPair& B) const;
  bool operator!=(const CameraParametersPair& B) const;

 private:
  std::shared_ptr<InputCameraParameters> input_ptr_;
  std::shared_ptr<OutputCameraParameters> output_ptr_;

  DistortionProcessing distortion_processing_;

  static constexpr double kFocalLengthEstimationAttempts = 100;
};

// holds the camera parameters of the first and second camera and uses them to
// generate virtual output cameras with properties that will produce correctly
// rectified images
class StereoCameraParameters {
 public:
  StereoCameraParameters(const double scale = 1.0);

  // 3. 方法签名修改 (NodeHandle -> Node*)
  bool setInputCameraParameters(rclcpp::Node* node,
                                const std::string& camera_namespace,
                                const CameraSide& side, const bool invert_T);

  // 4. 消息类型修改
  bool setInputCameraParameters(const sensor_msgs::msg::CameraInfo& camera_info,
                                const CameraSide& side);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 3>& K,
                                const std::vector<double>& D,
                                const DistortionModel& distortion_model,
                                const CameraSide& side);

  // 4. 消息类型修改
  void generateCameraInfoMessage(const CameraSide& side, const CameraIO& io,
                                 sensor_msgs::msg::CameraInfo* camera_info) const;

  const CameraParametersPair& getFirst() const;
  const CameraParametersPair& getSecond() const;

  bool valid() const;
  bool valid(const CameraSide& side, const CameraIO& io) const;

 private:
  bool generateRectificationParameters();

  double scale_;
  CameraParametersPair first_;
  CameraParametersPair second_;
};
}  // namespace image_undistort
#endif  // CAMERA_PARAMETERS_H