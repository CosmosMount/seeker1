cmake_minimum_required(VERSION 3.8)

project(disparity_image_proc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

add_library(disparity_component SHARED
  src/nodelets/depth_image.cpp
)


target_include_directories(disparity_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(disparity_component
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(disparity_component
  rclcpp
  rclcpp_components
  cv_bridge
  image_geometry
  image_transport
  message_filters
  sensor_msgs
  stereo_msgs
)

rclcpp_components_register_node(
  disparity_component
  PLUGIN "stereo_image_proc::DepthImageNodelet"
  EXECUTABLE disparity_image_proc_node
)

install(TARGETS
  disparity_component
  disparity_image_proc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



