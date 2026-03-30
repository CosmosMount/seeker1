cmake_minimum_required(VERSION 3.8)
project(image_undistort)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

# --- 【关键修改 1：修正模块搜索路径】 ---
# 使用 CMAKE_CURRENT_SOURCE_DIR 指向当前包的目录，而不是 CMAKE_SOURCE_DIR
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# --- 【关键修改 2：将 NLopt 设为必需】 ---
# 添加 REQUIRED 关键字，这样如果找不到 NLopt，编译会立即报错，方便调试
find_package(NLOPT REQUIRED)

set(ROS_DEPS
  rclcpp rclcpp_components rcl_interfaces sensor_msgs geometry_msgs
  cv_bridge image_transport pcl_ros tf2_ros tf2_eigen pcl_conversions
)

# --- 2. 基础库 (Core Library) ---
set(LIB_SRC
  src/stereo_info.cpp
  src/stereo_undistort.cpp
  src/depth.cpp
  src/camera_parameters.cpp
  src/image_undistort.cpp
  src/undistorter.cpp
  src/image_concat.cpp
  # 因为 NLOPT 现在是 REQUIRED，所以 NLOPT_FOUND 总是为真，可以直接添加源文件
  src/point_to_bearing.cpp
)

add_library(image_undistort_lib ${LIB_SRC})

ament_target_dependencies(image_undistort_lib ${ROS_DEPS})

# 将所有外部库链接合并到一起，更清晰
target_link_libraries(image_undistort_lib
  ${OpenCV_LIBRARIES}
  ${NLOPT_LIBRARIES}
)

# 将所有外部 include 目录合并到一起
target_include_directories(image_undistort_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # 使用 NLOPT_INCLUDE_DIRS 即可 (通常Find模块会提供复数形式的变量)
  ${NLOPT_INCLUDE_DIRS}
)

# --- 3. 组件库 (Nodelets) ---
add_library(image_undistort_component src/nodelets/image_undistort_nodelet.cpp)
target_link_libraries(image_undistort_component image_undistort_lib)
ament_target_dependencies(image_undistort_component ${ROS_DEPS})

add_library(stereo_info_component src/nodelets/stereo_info_nodelet.cpp)
target_link_libraries(stereo_info_component image_undistort_lib)
ament_target_dependencies(stereo_info_component ${ROS_DEPS})

add_library(stereo_undistort_component src/nodelets/stereo_undistort_nodelet.cpp)
target_link_libraries(stereo_undistort_component image_undistort_lib)
ament_target_dependencies(stereo_undistort_component ${ROS_DEPS})

add_library(depth_component src/nodelets/depth_nodelet.cpp)
target_link_libraries(depth_component image_undistort_lib)
ament_target_dependencies(depth_component ${ROS_DEPS})

add_library(image_concat_component src/nodelets/image_concat_nodelet.cpp)
target_link_libraries(image_concat_component image_undistort_lib)
ament_target_dependencies(image_concat_component ${ROS_DEPS})

# 因为 NLOPT 是 REQUIRED, if 条件可以移除
add_library(point_to_bearing_component src/nodelets/point_to_bearing_nodelet.cpp)
target_link_libraries(point_to_bearing_component image_undistort_lib)
ament_target_dependencies(point_to_bearing_component ${ROS_DEPS})

# --- 4. 注册组件 ---
# ... (这部分您的代码是OK的，无需改动) ...
rclcpp_components_register_node(image_undistort_component PLUGIN "image_undistort::ImageUndistortNodelet" EXECUTABLE image_undistort_node)
rclcpp_components_register_node(stereo_info_component PLUGIN "image_undistort::StereoInfoNodelet" EXECUTABLE stereo_info_node)
rclcpp_components_register_node(stereo_undistort_component PLUGIN "image_undistort::StereoUndistortNodelet" EXECUTABLE stereo_undistort_node)
rclcpp_components_register_node(depth_component PLUGIN "image_undistort::DepthNodelet" EXECUTABLE depth_node)
rclcpp_components_register_node(image_concat_component PLUGIN "image_undistort::ImageConcatNodelet" EXECUTABLE image_concat_node)
rclcpp_components_register_node(point_to_bearing_component PLUGIN "image_undistort::PointToBearingNodelet" EXECUTABLE point_to_bearing_node)

# --- 5. 安装规则 ---
# ... (这部分您的代码是OK的，可以稍微简化) ...
set(INSTALL_TARGETS
    image_undistort_lib
    image_undistort_component
    stereo_info_component
    stereo_undistort_component
    depth_component
    image_concat_component
    point_to_bearing_component
    
    image_undistort_node
    stereo_info_node
    stereo_undistort_node
    depth_node
    image_concat_node
    point_to_bearing_node
)

install(TARGETS ${INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY cmake/ DESTINATION share/${PROJECT_NAME}/cmake)

ament_package()