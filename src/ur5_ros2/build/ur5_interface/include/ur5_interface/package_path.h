#pragma once

#include <cstddef>
#include <string>

namespace arm_interface {

/** Gets the path to the package source directory. */
inline std::string getPath() {
  return "/root/ocs2_ros2/ros_ws/src/ur5_ros2/ur5_interface";
}

}  // namespace mobile_manipulator
