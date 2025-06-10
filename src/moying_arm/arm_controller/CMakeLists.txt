cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  std_msgs
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_mobile_manipulator
  arm_interface
  geometry_msgs
  ament_index_cpp
  localization_msgs
  Boost
  tf2
  hpp-fcl
  pinocchio
  angles
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(arm_interface REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
list(APPEND CMAKE_PREFIX_PATH "/root/ocs2_ros2/ros_ws/install/localization_msgs")
  find_package(${Dependency} REQUIRED)
endforeach()

# Add arm_controller library related compile commands
generate_parameter_library(arm_controller_parameters
  src/arm_controller.yaml
  include/arm_controller/validate_arm_controller_parameters.hpp
)

include_directories(
  include
  ${ament_cmake_INCLUDE_DIRS}
  ${arm_interface_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/arm_controller.cpp
)
target_include_directories(arm_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(arm_controller arm_controller_parameters arm_interface::arm_interface
${ur5_interface_LIBRARIES}
hpp-fcl)
ament_target_dependencies(arm_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(arm_controller PRIVATE "ARM_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface arm_controller.xml)

install(
  TARGETS
  arm_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  arm_controller
)

ament_package()
