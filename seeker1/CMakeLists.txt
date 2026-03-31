# 1. 设置 CMake 版本并声明项目
cmake_minimum_required(VERSION 3.8)
project(seeker)

# 2. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(stereo_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(USB-1.0 REQUIRED libusb-1.0)

set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs")


add_library(seeker_sdk SHARED IMPORTED)
set_target_properties(seeker_sdk PROPERTIES
  IMPORTED_LOCATION "${LIB_DIR}/libseeker.so"
)

# 4. 添加核心组件库
add_library(seeker_nodelet SHARED
  src/seeker_nodelet.cpp
)

# [修正点 1] 链接 ROS 依赖 (ament_target_dependencies 默认是 PUBLIC/INTERFACE 混合的 plain 模式)
ament_target_dependencies(seeker_nodelet
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  stereo_msgs
)

# [修正点 2] 链接非 ROS 依赖 (移除 PRIVATE 关键字，以匹配上面的 plain 模式)
target_link_libraries(seeker_nodelet
  seeker_sdk
  ${OpenCV_LIBRARIES}
  ${USB-1.0_LIBRARIES}
)

# 设置 include 目录
target_include_directories(seeker_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# [修正点 3] 移除错误的 include(RclcppComponents)

# [修正点 4] 添加 EXECUTABLE 参数
# 这会自动生成一个名为 seeker_node 的可执行文件，加载我们的组件
rclcpp_components_register_node(
  seeker_nodelet
  PLUGIN "SeekRosNode"
  EXECUTABLE seeker_node
)

# 5. 编译 PPS 工具
add_executable(pps_s_main pps/pps_s_main.c)

# 6. 安装规则
install(
  TARGETS 
    seeker_nodelet
    seeker_node  # 安装自动生成的节点可执行文件
    pps_s_main   # 安装 PPS 工具
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # 安装到 lib/seeker/ 下，方便 ros2 run
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ${LIB_DIR}/libseeker.so
  DESTINATION lib
)

install(DIRECTORY launch/ros2
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()