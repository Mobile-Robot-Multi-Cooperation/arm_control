#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <functional>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <memory>
#include <mutex>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace ur5_interface{
using namespace ocs2;
/**
 * This class lets the user to command robot form interactive marker.
 */
class UR5JoyTargetTrajectories final {
 public:
  using GaolPoseToTargetTrajectories = std::function<ocs2::TargetTrajectories(
      const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
      const SystemObservation& observation)>;


  /**
   * Constructor
   *
   * @param [in] node: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on
   * "topicPrefix_mpc_target" topic. Moreover, the latest observation is be
   * expected on "topicPrefix_mpc_observation" topic.
   * @param [in] gaolPoseToTargetTrajectories: A function which transforms the
   * commanded pose to TargetTrajectories.
   */
  UR5JoyTargetTrajectories(
      const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
      GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,std::string taskFile);

  /**
   * Spins ROS to update the interactive markers.
   */
  void publishInteractiveMarker() { rclcpp::spin(node_); }

 private:
  void timer_callback();
  visualization_msgs::msg::InteractiveMarker createInteractiveMarker(int mode) const;

  rclcpp::Node::SharedPtr node_;

  GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories_;

  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>
      targetTrajectoriesPublisherPtr_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_subsciption_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Joy>> joy_command_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_point_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>> goal_point_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr inc_goal_point_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Pose>> inc_goal_point_ptr_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool use_joy,use_cmd,close_end_eff;

  Eigen::Vector3d position_;
  Eigen::Vector3d init_position_;
  Eigen::Vector3d arm_position_;
  Eigen::Quaterniond orientation_;
  Eigen::Quaterniond init_orientation_;
};

}  