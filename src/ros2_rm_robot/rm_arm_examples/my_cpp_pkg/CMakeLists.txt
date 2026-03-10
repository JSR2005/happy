cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(rm_driver REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rm_force_position_new src/api_Force_Position_Control_demo.cpp)

# 新增：添加编译选项，抑制unused-function警告
target_compile_options(rm_force_position_new PRIVATE
  -Wno-unused-function  # 屏蔽未使用函数的警告
  -Wno-unused-variable  # 可选：顺便屏蔽之前的unused-variable警告
)

# 1. 添加头文件包含路径（找到rm_interface_global.h）
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  /home/jsr/ros2_ws/src/ros2_rm_robot/rm_driver/include
)

ament_target_dependencies(rm_force_position_new rclcpp std_msgs rm_ros_interfaces rm_driver  sensor_msgs geometry_msgs)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
rm_force_position_new 
  DESTINATION lib/${PROJECT_NAME})
ament_package()