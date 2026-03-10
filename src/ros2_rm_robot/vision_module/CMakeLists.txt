cmake_minimum_required(VERSION 3.8)
project(vision_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vision_interfaces REQUIRED)

# Include directories
include_directories(include)

# YOLOv8 检测节点
add_library(yolo_detector_component
  src/yolo_detector.cpp)

target_include_directories(yolo_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(yolo_detector_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(yolo_detector_component
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  OpenCV
  vision_interfaces)

rclcpp_components_register_node(yolo_detector_component
  PLUGIN "vision_module::YOLODetectorNode"
  EXECUTABLE yolo_detector_node)

# 手眼标定节点
add_library(hand_eye_calibration_component
  src/hand_eye_calibration.cpp)

target_include_directories(hand_eye_calibration_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(hand_eye_calibration_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(hand_eye_calibration_component
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  OpenCV
  Eigen3)

rclcpp_components_register_node(hand_eye_calibration_component
  PLUGIN "vision_module::HandEyeCalibrationNode"
  EXECUTABLE hand_eye_calibration_node)

# 骨骼检测节点 (如果有实现)
# add_library(skeleton_detector_component
#   src/skeleton_detector.cpp)
# ...

# 安装
install(TARGETS
  yolo_detector_component
  hand_eye_calibration_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  yolo_detector_node
  hand_eye_calibration_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_common REQUIRED)
  # ament_lint_auto_find_test_packages()
endif()

ament_package()
