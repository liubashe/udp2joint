cmake_minimum_required(VERSION 3.8)
project(udp2joint)

# 使用 C++14 或更高版本
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找必要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# 包含头文件目录
include_directories(
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${controller_manager_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(dlltest
  src/dlltest.cpp
  src/MySocketUdp.cpp
  src/PositionB_FK.cpp
  src/PositionH_FK.cpp
  src/stdafx.cpp
)
ament_target_dependencies(dlltest
  rclcpp
  std_msgs
)

# add_executable(udp2joint
#   src/udp2joint.cpp
#   src/MySocketUdp.cpp
#   src/PositionB_FK.cpp
#   src/PositionH_FK.cpp
#   src/stdafx.cpp
# )
# ament_target_dependencies(udp2joint
#   rclcpp
#   std_msgs
# )

add_executable(moveit_test
  src/moveit_test.cpp
  src/MySocketUdp.cpp
  src/PositionB_FK.cpp
  src/PositionH_FK.cpp
  src/stdafx.cpp
)

# 为moveit_test添加所有需要的依赖
ament_target_dependencies(moveit_test
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  control_msgs
  trajectory_msgs
  rclcpp_action
  controller_manager_msgs
)

# 安装目标
install(TARGETS
  dlltest
  # udp2joint
  moveit_test
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件（如果有）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 使用 ament 打包
ament_package()

# # 添加编译目标的依赖
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   std_msgs
#   sensor_msgs
#   moveit_core
#   moveit_ros_planning_interface
#   control_msgs
#   trajectory_msgs
#   rclcpp_action
# )
