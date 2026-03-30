#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"
#include <rclcpp/rclcpp.hpp>

namespace image_undistort {

// [新] 辅助函数，用于将 ROS 2 参数 (扁平 vector) 转换为 Eigen 矩阵
template <typename Derived>
static void vectorToMatrix(const std::vector<double>& vec,
                           Eigen::MatrixBase<Derived>* output) {
  if (vec.size() != static_cast<size_t>(output->rows() * output->cols())) {
    throw std::runtime_error("Loaded matrix vector has incorrect number of elements. Expected: " +
                             std::to_string(output->rows() * output->cols()) +
                             ", Got: " + std::to_string(vec.size()));
  }
  for (int i = 0; i < output->rows(); ++i) {
    for (int j = 0; j < output->cols(); ++j) {
      output->coeffRef(i, j) = vec[i * output->cols() + j];
    }
  }
}

// 1. [签名迁移] 构造函数接收 rclcpp::Node*
BaseCameraParameters::BaseCameraParameters(rclcpp::Node* node,
                                           const std::string& camera_namespace,
                                           const bool invert_T) {
  RCLCPP_INFO(node->get_logger(), "Loading camera parameters for namespace: %s", camera_namespace.c_str());

  // 2. [参数迁移] ROS 2 API 替换 XmlRpc
  // ROS 2 要求先声明参数。我们声明所有可能的参数，默认值为空 vector。
  node->declare_parameter<std::vector<double>>(camera_namespace + "/K", {});
  node->declare_parameter<std::vector<double>>(camera_namespace + "/intrinsics", {});
  node->declare_parameter<std::vector<double>>(camera_namespace + "/resolution", {});
  node->declare_parameter<std::vector<double>>(camera_namespace + "/T_cn_cnm1", {});
  node->declare_parameter<std::vector<double>>(camera_namespace + "/T", {});
  node->declare_parameter<std::vector<double>>(camera_namespace + "/P", {});
  
  // 现在获取已声明的参数
  std::vector<double> K_in = node->get_parameter(camera_namespace + "/K").as_double_array();
  std::vector<double> intrinsics_in = node->get_parameter(camera_namespace + "/intrinsics").as_double_array();

  bool K_loaded = !K_in.empty();
  if (K_loaded) {
    vectorToMatrix(K_in, &K_); // 使用新的辅助函数
  }

  if (!intrinsics_in.empty()) {
    if (K_loaded) {
      RCLCPP_WARN(
          node->get_logger(),
          "Both K and intrinsics vector given, ignoring intrinsics "
          "vector");
    } else if (intrinsics_in.size() < 4) {
      throw std::runtime_error(
          "Intrinsics vector must have at least 4 values (Fx,Fy,Cx,Cy)");
    }

    K_ = Eigen::Matrix3d::Identity();
    K_(0, 0) = intrinsics_in[intrinsics_in.size() - 4];
    K_(1, 1) = intrinsics_in[intrinsics_in.size() - 3];
    K_(0, 2) = intrinsics_in[intrinsics_in.size() - 2];
    K_(1, 2) = intrinsics_in[intrinsics_in.size() - 1];
  } else if (!K_loaded) {
    throw std::runtime_error("Could not find K or camera intrinsics vector");
  }

  std::vector<double> resolution_in = node->get_parameter(camera_namespace + "/resolution").as_double_array();
  if (!resolution_in.empty()) {
    if (resolution_in.size() != 2) {
      throw std::runtime_error("Resolution must have exactly 2 values (x,y)");
    }
    resolution_.width = static_cast<int>(resolution_in[0]);
    resolution_.height = static_cast<int>(resolution_in[1]);
  } else {
    throw std::runtime_error("Could not find camera resolution");
  }

  std::vector<double> T_in = node->get_parameter(camera_namespace + "/T_cn_cnm1").as_double_array();
  if (T_in.empty()) {
      T_in = node->get_parameter(camera_namespace + "/T").as_double_array();
  }

  bool T_loaded = !T_in.empty();
  if (T_loaded) {
    vectorToMatrix(T_in, &T_);
  } else {
    T_ = Eigen::Matrix4d::Identity();
  }

  if (invert_T) {
    T_ = T_.inverse();
  }

  std::vector<double> P_in = node->get_parameter(camera_namespace + "/P").as_double_array();
  if (!P_in.empty()) {
    vectorToMatrix(P_in, &P_);

    if (!T_loaded) {
      T_.topLeftCorner<3, 3>() = K_.inverse() * P_.topLeftCorner<3, 3>();
      T_.topRightCorner<3, 1>() = K_.inverse() * P_.topRightCorner<3, 1>();
      T_(3, 3) = 1;
    } else if (!P_.isApprox((Eigen::Matrix<double, 3, 4>() << K_,
                                Eigen::Vector3d::Constant(0))
                                   .finished() *
                               T_)) {
      RCLCPP_WARN_ONCE(
          node->get_logger(),
          "For given K, T and P ([K,[0;0;0]]*T != P), replacing K with "
          "corrected value");
      K_ = P_.topLeftCorner<3, 3>() * T_.topLeftCorner<3, 3>().inverse();
    }
  } else {
    P_ = (Eigen::Matrix<double, 3, 4>() << K_, Eigen::Vector3d::Constant(0))
             .finished() *
         T_;
  }
}

// 3. [消息类型迁移]
BaseCameraParameters::BaseCameraParameters(
    const sensor_msgs::msg::CameraInfo& camera_info) {
  resolution_.height = camera_info.height;
  resolution_.width = camera_info.width;

  K_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      camera_info.k.data());

  T_ = Eigen::Matrix4d::Identity();
  T_.topLeftCorner<3, 3>() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          camera_info.r.data());

  P_ = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
      camera_info.p.data());

  T_.topRightCorner<3, 1>() = K_.inverse() * P_.topRightCorner<3, 1>();

  if (!P_.topLeftCorner<3, 3>().isApprox(K_ * T_.topLeftCorner<3, 3>())) {
    // 4. [日志迁移]
    RCLCPP_WARN_ONCE(
        rclcpp::get_logger("camera_parameters"), // 使用一个静态 logger
        "For given K, T and P ([K,[0;0;0]]*T != P), replacing K with corrected "
        "value");
    K_ = P_.topLeftCorner<3, 3>() * T_.topLeftCorner<3, 3>().inverse();
  }
}

// ... (纯 Eigen/C++ 逻辑, 无需修改) ...
BaseCameraParameters::BaseCameraParameters(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_in,
    const Eigen::Matrix<double, 3, 3>& K_in)
    : resolution_(resolution_in),
      T_(T_in),
      P_((Eigen::Matrix<double, 3, 4>() << K_in, Eigen::Vector3d::Constant(0))
             .finished() *
         T_in),
      K_(K_in) {}
const cv::Size& BaseCameraParameters::resolution() const { return resolution_; }
const Eigen::Matrix<double, 4, 4>& BaseCameraParameters::T() const {
  return T_;
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> BaseCameraParameters::R()
    const {
  return T_.topLeftCorner<3, 3>();
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> BaseCameraParameters::p()
    const {
  return T_.topRightCorner<3, 1>();
}
const Eigen::Matrix<double, 3, 4>& BaseCameraParameters::P() const {
  return P_;
}
const Eigen::Matrix<double, 3, 3>& BaseCameraParameters::K() const {
  return K_;
}
bool BaseCameraParameters::operator==(const BaseCameraParameters& B) const {
  return (resolution() == B.resolution()) && (T() == B.T()) && (P() == B.P()) &&
         (K() == B.K());
}
bool BaseCameraParameters::operator!=(const BaseCameraParameters& B) const {
  return !(*this == B);
}
// ... (结束 纯 Eigen/C++ 逻辑) ...


// 1. [签名迁移]
InputCameraParameters::InputCameraParameters(
    rclcpp::Node* node, const std::string& camera_namespace,
    const bool invert_T)
    : BaseCameraParameters(node, camera_namespace, invert_T) { // 3. 传递 Node*
  
  // 4. [参数迁移] 替换 nh.getParam, 使用 get_parameter_or (声明并获取)
  std::string distortion_model_in = 
      node->get_parameter_or<std::string>(camera_namespace + "/distortion_model", "");
  std::string camera_model_in = 
      node->get_parameter_or<std::string>(camera_namespace + "/camera_model", "");

  if (distortion_model_in.empty() || camera_model_in.empty()) {
    RCLCPP_WARN(
        node->get_logger(),
        "No camera and/or distortion model given, assuming pinhole-radtan");
    distortion_model_ = DistortionModel::RADTAN;
  } else {
    distortion_model_ =
        stringToDistortion(distortion_model_in, camera_model_in);
  }

  std::vector<double> intrinsics_in = 
      node->get_parameter_or<std::vector<double>>(camera_namespace + "/intrinsics", {});
  
  if (intrinsics_in.size() > 4) {
    D_.push_back(intrinsics_in[0]);
  }
  if (intrinsics_in.size() > 5) {
    D_.push_back(intrinsics_in[1]);
  }
  if (intrinsics_in.size() > 6) {
    throw std::runtime_error(
        "Intrinsics vector cannot have more than 6 values");
  }

  std::vector<double> D_in =
      node->get_parameter_or<std::vector<double>>(camera_namespace + "/distortion_coeffs", {});
  
  if (!D_in.empty()) {
    D_.insert(D_.end(), D_in.begin(), D_in.end());
  }

  if (D_.empty()) {
    RCLCPP_WARN(
        node->get_logger(),
        "No distortion coefficients found, assuming images are "
        "undistorted");
  }

  // ensure D always has at least 7 elements
  while (D_.size() < 7) {
    D_.push_back(0);
  }
}

// 3. [消息类型迁移]
InputCameraParameters::InputCameraParameters(
    const sensor_msgs::msg::CameraInfo& camera_info)
    : BaseCameraParameters(camera_info),
      D_(camera_info.d),
      distortion_model_(
          stringToDistortion(camera_info.distortion_model, "pinhole")) {
  // ensure D always has at least 5 elements
  while (D_.size() < 5) {
    D_.push_back(0);
  }
}

// ... (纯 C++ 逻辑, 无需修改) ...
InputCameraParameters::InputCameraParameters(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_in,
    const Eigen::Matrix<double, 3, 3>& K_in, const std::vector<double>& D_in,
    const DistortionModel& distortion_model)
    : BaseCameraParameters(resolution_in, T_in, K_in),
      D_(D_in),
      distortion_model_(distortion_model) {
  while (D_.size() < 5) {
    D_.push_back(0);
  }
}
const std::vector<double>& InputCameraParameters::D() const { return D_; }
const DistortionModel& InputCameraParameters::distortionModel() const {
  return distortion_model_;
}
const DistortionModel InputCameraParameters::stringToDistortion(
    const std::string& distortion_model, const std::string& camera_model) {
  std::string lower_case_distortion_model = distortion_model;
  std::string lower_case_camera_model = camera_model;

  std::transform(lower_case_distortion_model.begin(),
                 lower_case_distortion_model.end(),
                 lower_case_distortion_model.begin(), ::tolower);
  std::transform(lower_case_camera_model.begin(), lower_case_camera_model.end(),
                 lower_case_camera_model.begin(), ::tolower);

  if (lower_case_camera_model == "pinhole") {
    if (lower_case_camera_model == std::string("none")) {
      return DistortionModel::NONE;
    } else if ((lower_case_distortion_model == std::string("plumb bob")) ||
               (lower_case_distortion_model == std::string("plumb_bob")) ||
               (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::RADTAN;
    } else if (lower_case_distortion_model == std::string("equidistant")) {
      return DistortionModel::EQUIDISTANT;
    } else if (lower_case_distortion_model == std::string("fov")) {
      return DistortionModel::FOV;
    } else {
      throw std::runtime_error(
          "Unrecognized distortion model for pinhole camera. Valid pinhole "
          "distortion model options are 'none', 'radtan', 'Plumb Bob', "
          "'plumb_bob', "
          "'equidistant' and 'fov'.");
    }
  } else if (lower_case_camera_model == "omni") {
    if ((lower_case_distortion_model == std::string("plumb bob")) ||
        (lower_case_distortion_model == std::string("plumb_bob")) ||
        (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::OMNIRADTAN;
    } else if (lower_case_distortion_model == std::string("none")) {
      return DistortionModel::OMNI;
    } else {
      throw std::runtime_error(
          "Unrecognized distortion model for omni camera. Valid omni "
          "distortion model options are 'none' and 'radtan'.");
    }
  } else if ((lower_case_camera_model == std::string("double_sphere")) ||
             (lower_case_camera_model == std::string("ds"))) {
    return DistortionModel::DOUBLESPHERE;
  } else if (lower_case_camera_model == std::string("unified")) {
    return DistortionModel::UNIFIED;
  } else if ((lower_case_camera_model == std::string("extended_unified")) ||
             (lower_case_camera_model == std::string("eucm"))) {
    return DistortionModel::EXTENDEDUNIFIED;
  } else {
    throw std::runtime_error(
        "Unrecognized camera model. Valid camera models are 'pinhole', "
        "'omni', 'double_sphere', 'ds', 'unified', 'extended_unified' and "
        "'eucm'");
  }
}
bool InputCameraParameters::operator==(const InputCameraParameters& B) const {
  return (*dynamic_cast<const BaseCameraParameters*>(this) == B) &&
         (D() == B.D()) && (distortionModel() == B.distortionModel());
}
bool InputCameraParameters::operator!=(const InputCameraParameters& B) const {
  return !(*this == B);
}
CameraParametersPair::CameraParametersPair(
    const DistortionProcessing distortion_processing)
    : distortion_processing_(distortion_processing){};
// ... (结束 纯 C++ 逻辑) ...


// 1. [签名迁移]
bool CameraParametersPair::setCameraParameters(
    rclcpp::Node* node, const std::string& camera_namespace,
    const CameraIO& io, const bool invert_T) {
  try {
    if (io == CameraIO::INPUT) {
      // 3. 传递 Node*
      input_ptr_ = std::make_shared<InputCameraParameters>(node, camera_namespace,
                                                         invert_T);
    } else {
      output_ptr_ = std::make_shared<OutputCameraParameters>(
          node, camera_namespace, invert_T);
    }
    return true;
  } catch (const std::runtime_error& e) { // 4. 捕获 const 引用
    // 5. [日志迁移]
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    return false;
  }
}

// 3. [消息类型迁移]
bool CameraParametersPair::setCameraParameters(
    const sensor_msgs::msg::CameraInfo& camera_info, const CameraIO& io) {
  try {
    if (io == CameraIO::INPUT) {
      input_ptr_ = std::make_shared<InputCameraParameters>(camera_info);
    } else {
      output_ptr_ = std::make_shared<OutputCameraParameters>(camera_info);
    }
    return true;
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"), "%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model) {
  try {
    input_ptr_ = std::make_shared<InputCameraParameters>(resolution, T, K, D,
                                                         distortion_model);
    return true;
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"), "%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setOutputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K) {
  try {
    output_ptr_ = std::make_shared<OutputCameraParameters>(resolution, T, K);
    return true;
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"), "%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setOutputFromInput(const double scale) {
  if (!valid(CameraIO::INPUT)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("camera_parameters"),
        "Cannot set output to same values as input, as input is not currently "
        "set");
    return false;
  } else {
    Eigen::Matrix<double, 3, 3> K = input_ptr_->K();
    K(0, 0) *= scale;
    K(1, 1) *= scale;
    setOutputCameraParameters(input_ptr_->resolution(), input_ptr_->T(), K);
    return true;
  }
}

bool CameraParametersPair::setOptimalOutputCameraParameters(
    const double scale) {
  if (!valid(CameraIO::INPUT)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("camera_parameters"),
        "Optimal output camera parameters cannot be set until the input camera "
        "parameters have been given");
    return false;
  }
  // ... (纯 OpenCV/Eigen 逻辑, 无需修改) ...
  cv::Size resolution_estimate(
      std::ceil(input_ptr_->resolution().width * scale),
      std::ceil(input_ptr_->resolution().height * scale));
  double focal_length =
      scale * (input_ptr_->K()(0, 0) + input_ptr_->K()(1, 1)) / 2;
  Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
  P(0, 0) = focal_length;
  P(1, 1) = focal_length;
  P(2, 2) = 1;
  P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
  P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
  P.topRightCorner<3, 1>() = focal_length * input_ptr_->p();
  std::vector<double> D;
  if (distortion_processing_ == DistortionProcessing::UNDISTORT) {
    D = input_ptr_->D();
  } else {
    D = std::vector<double>(7, 0);
  }
  for (size_t i = 0; i < kFocalLengthEstimationAttempts; ++i) {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
        pixel_locations;
    for (size_t v = 0; v < resolution_estimate.height; ++v) {
      pixel_locations.emplace_back(0, v);
      pixel_locations.emplace_back(resolution_estimate.width - 1, v);
    }
    for (size_t u = 1; u < (resolution_estimate.width - 1); ++u) {
      pixel_locations.emplace_back(u, 0);
      pixel_locations.emplace_back(u, resolution_estimate.height - 1);
    }
    double max_x = 0;
    double max_y = 0;
    for (Eigen::Vector2d pixel_location : pixel_locations) {
      Eigen::Vector2d distorted_pixel_location;
      Undistorter::distortPixel(input_ptr_->K(), input_ptr_->R(), P,
                                input_ptr_->distortionModel(), D,
                                pixel_location, &distorted_pixel_location);

      max_x = std::max(
          max_x,
          std::abs(static_cast<double>(input_ptr_->resolution().width) / 2.0 -
                   distorted_pixel_location.x()));
      max_y = std::max(
          max_y,
          std::abs(static_cast<double>(input_ptr_->resolution().height) / 2.0 -
                   distorted_pixel_location.y()));
    }
    cv::Size resolution_update;
    resolution_update.width = std::floor(
        static_cast<double>(resolution_estimate.width) *
        std::abs(static_cast<double>(input_ptr_->resolution().width) - max_x) /
        (static_cast<double>(input_ptr_->resolution().width) / 2.0));
    resolution_update.height = std::floor(
        static_cast<double>(resolution_estimate.height) *
        std::abs(static_cast<double>(input_ptr_->resolution().height) - max_y) /
        (static_cast<double>(input_ptr_->resolution().height) / 2.0));
    if (resolution_update == resolution_estimate) {
      break;
    } else {
      resolution_estimate = resolution_update;
      P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
      P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
    }
  }
  Eigen::Matrix3d K = P.topLeftCorner<3, 3>();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topRightCorner<3, 1>() = input_ptr_->p();
  return setOutputCameraParameters(resolution_estimate, T, K);
}

// 3. [消息类型迁移]
void CameraParametersPair::generateCameraInfoMessage(
    const CameraIO& io, sensor_msgs::msg::CameraInfo* camera_info) const {
  if (!valid()) {
    throw std::runtime_error(
        "Attempted to get output camera_info before a valid input and output "
        "has been set");
  } else {
    std::shared_ptr<BaseCameraParameters> camera_parameters_ptr;
    if (io == CameraIO::INPUT) {
      camera_parameters_ptr = input_ptr_;
    } else {
      camera_parameters_ptr = output_ptr_;
    }

    camera_info->height = camera_parameters_ptr->resolution().height;
    camera_info->width = camera_parameters_ptr->resolution().width;

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        camera_info->k.data()) = camera_parameters_ptr->K();

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        camera_info->r.data()) = camera_parameters_ptr->R();

    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
        camera_info->p.data()) = camera_parameters_ptr->P();

    if (io == CameraIO::OUTPUT ||
        distortion_processing_ == DistortionProcessing::UNDISTORT) {
      camera_info->d = std::vector<double>(5, 0);
    } else {
      camera_info->d = input_ptr_->D();
    }
    if (input_ptr_->distortionModel() == DistortionModel::RADTAN) {
      camera_info->distortion_model = "radtan";
    } else {
      camera_info->distortion_model = "equidistant";
    }
  }
}

// ... (纯 C++ 逻辑, 无需修改) ...
const DistortionProcessing& CameraParametersPair::distortionProcessing() const {
  return distortion_processing_;
}
const std::shared_ptr<InputCameraParameters>&
CameraParametersPair::getInputPtr() const {
  return input_ptr_;
}
const std::shared_ptr<OutputCameraParameters>&
CameraParametersPair::getOutputPtr() const {
  return output_ptr_;
}
bool CameraParametersPair::valid() const {
  return (input_ptr_ != nullptr) && (output_ptr_ != nullptr);
}
bool CameraParametersPair::valid(const CameraIO& io) const {
  if (io == CameraIO::INPUT) {
    return input_ptr_ != nullptr;
  } else {
    return output_ptr_ != nullptr;
  }
}
bool CameraParametersPair::operator==(const CameraParametersPair& B) const {
  return *getInputPtr() == *B.getInputPtr() &&
         (*getOutputPtr() == *B.getOutputPtr());
}
bool CameraParametersPair::operator!=(const CameraParametersPair& B) const {
  return !(*this == B);
}
StereoCameraParameters::StereoCameraParameters(const double scale)
    : scale_(scale){};
// ... (结束 纯 C++ 逻辑) ...


// 1. [签名迁移]
bool StereoCameraParameters::setInputCameraParameters(
    rclcpp::Node* node, const std::string& camera_namespace,
    const CameraSide& side, const bool invert_T) {
  bool success;
  if (side == CameraSide::FIRST) {
    // 3. 传递 Node*
    success = first_.setCameraParameters(node, camera_namespace, CameraIO::INPUT,
                                        invert_T);
  } else {
    success = second_.setCameraParameters(node, camera_namespace, CameraIO::INPUT,
                                         invert_T);
  }
  if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
      valid(CameraSide::SECOND, CameraIO::INPUT)) {
    generateRectificationParameters();
  }
  return success;
}

// 3. [消息类型迁移]
bool StereoCameraParameters::setInputCameraParameters(
    const sensor_msgs::msg::CameraInfo& camera_info, const CameraSide& side) {
  try {
    if (side == CameraSide::FIRST) {
      first_.setCameraParameters(camera_info, CameraIO::INPUT);
    } else {
      second_.setCameraParameters(camera_info, CameraIO::INPUT);
    }
    if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
        valid(CameraSide::SECOND, CameraIO::INPUT)) {
      generateRectificationParameters();
    }
    return true;
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"), "%s", e.what());
    return false;
  }
}

bool StereoCameraParameters::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model, const CameraSide& side) {
  try {
    if (side == CameraSide::FIRST) {
      first_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    } else {
      second_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    }
    if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
        valid(CameraSide::SECOND, CameraIO::INPUT)) {
      generateRectificationParameters();
    }
    return true;
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"), "%s", e.what());
    return false;
  }
}

// 3. [消息类型迁移]
void StereoCameraParameters::generateCameraInfoMessage(
    const CameraSide& side, const CameraIO& io,
    sensor_msgs::msg::CameraInfo* camera_info) const {
  if (side == CameraSide::FIRST) {
    first_.generateCameraInfoMessage(io, camera_info);
  } else {
    second_.generateCameraInfoMessage(io, camera_info);
  }
}

// ... (纯 C++ 逻辑, 无需修改) ...
bool StereoCameraParameters::valid() const {
  return first_.valid() && second_.valid();
}
bool StereoCameraParameters::valid(const CameraSide& side,
                                   const CameraIO& io) const {
  if (side == CameraSide::FIRST) {
    return first_.valid(io);
  } else {
    return second_.valid(io);
  }
}
// ... (结束 纯 C++ 逻辑) ...


bool StereoCameraParameters::generateRectificationParameters() {
  if (first_.getInputPtr()->p().isApprox(second_.getInputPtr()->p())) {
    RCLCPP_ERROR(
        rclcpp::get_logger("camera_parameters"),
        "Stereo rectification cannot be performed on cameras with a baseline "
        "of zero");
    return false;
  }

  // ... (纯 Eigen 逻辑, 无需修改) ...
  Eigen::Vector3d x = first_.getInputPtr()->p() - second_.getInputPtr()->p();
  Eigen::Vector3d y = first_.getInputPtr()->R().col(2).cross(x);
  Eigen::Vector3d z = x.cross(y);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();

  if (T(0, 0) < 0) {
    x = second_.getInputPtr()->p() - first_.getInputPtr()->p();
    y = second_.getInputPtr()->R().col(2).cross(x);
    z = x.cross(y);
    T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();
  }

  first_.setInputCameraParameters(
      first_.getInputPtr()->resolution(),
      T.inverse() * first_.getInputPtr()->T(), first_.getInputPtr()->K(),
      first_.getInputPtr()->D(), first_.getInputPtr()->distortionModel());
  second_.setInputCameraParameters(
      second_.getInputPtr()->resolution(),
      T.inverse() * second_.getInputPtr()->T(), second_.getInputPtr()->K(),
      second_.getInputPtr()->D(), second_.getInputPtr()->distortionModel());

  if (!first_.setOptimalOutputCameraParameters(scale_) ||
      !second_.setOptimalOutputCameraParameters(scale_)) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"),
                 "Automatic generation of stereo output parameters failed");
    return false;
  }
  // ... (结束 纯 Eigen 逻辑) ...

  cv::Size resolution(std::min(first_.getOutputPtr()->resolution().width,
                               second_.getOutputPtr()->resolution().width),
                      std::min(first_.getOutputPtr()->resolution().height,
                               second_.getOutputPtr()->resolution().height));

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = std::max(first_.getOutputPtr()->K()(0, 0),
                     second_.getOutputPtr()->K()(0, 0));
  K(1, 1) = K(0, 0);
  K(0, 2) = static_cast<double>(resolution.width) / 2.0;
  K(1, 2) = static_cast<double>(resolution.height) / 2.0;
  K(2, 2) = 1;

  if (!first_.setOutputCameraParameters(resolution, first_.getOutputPtr()->T(),
                                        K) ||
      !second_.setOutputCameraParameters(resolution,
                                         second_.getOutputPtr()->T(), K)) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_parameters"),
                 "Automatic generation of stereo output parameters failed");
    return false;
  }

  return true;
}

// ... (纯 C++ 逻辑, 无需修改) ...
const CameraParametersPair& StereoCameraParameters::getFirst() const {
  return first_;
}
const CameraParametersPair& StereoCameraParameters::getSecond() const {
  return second_;
}
// ... (结束 纯 C++ 逻辑) ...

}  // namespace image_undistort