cmake_minimum_required(VERSION 3.10)
project(modular_robot_sim)

include_directories(src/robot_physics/include)
include_directories(src/robot_sim_ros/include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# addition for ROS
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

#Define the library target
add_library(robot_physics_lib SHARED 
    src/robot_physics/src/engine.cpp
)

target_include_directories(robot_physics_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/robot_physics/include>
    ${EIGEN3_INCLUDE_DIRS}
)

# Build the ROS 2 Nodes
# This one command replaces all target_link_libraries and include_directories for ROS
add_executable(sim_node src/robot_sim_ros/src/sim_node.cpp)
ament_target_dependencies(sim_node rclcpp nav_msgs tf2_ros tf2)
target_link_libraries(sim_node robot_physics_lib Eigen3::Eigen)

add_executable(sensor_bridge src/robot_sim_ros/src/sensor_bridge.cpp)
ament_target_dependencies(sensor_bridge rclcpp nav_msgs tf2_ros tf2)
target_link_libraries(sensor_bridge robot_physics_lib Eigen3::Eigen)

add_executable(controller_node src/robot_sim_ros/src/controller.cpp)
ament_target_dependencies(controller_node rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
target_link_libraries(controller_node robot_physics_lib Eigen3::Eigen)

add_executable(teleop_node src/robot_sim_ros/src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp nav_msgs geometry_msgs sensor_msgs)
target_link_libraries(teleop_node robot_physics_lib Eigen3::Eigen)

add_executable(pose_relay_node src/robot_sim_ros/src/pose_relay_node.cpp)
ament_target_dependencies(pose_relay_node rclcpp nav_msgs ros_gz_interfaces)
target_link_libraries(pose_relay_node robot_physics_lib Eigen3::Eigen)

# Add the tests folder
add_subdirectory(test)

install(TARGETS
  sim_node
  sensor_bridge
  controller_node
  teleop_node
  pose_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/launch
   src/worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/robot_sim_ros/urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()