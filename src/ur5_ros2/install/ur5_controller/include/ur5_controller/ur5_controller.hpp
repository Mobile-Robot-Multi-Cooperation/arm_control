#ifndef UR5_CONTROLLER__UR5_CONTROLLER_HPP_
#define UR5_CONTROLLER__UR5_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
// #include "mmco_controller_parameters.hpp" // 不好改，暂存
// #include "arm_controller/visibility_control.h"
// #include <arm_interface/command/joy_target_trajectories.h>
#include "ur5_controller_parameters.hpp" // 不好改，暂存
#include "ur5_controller/visibility_control.h"
#include <ur5_interface/command/joy_target_trajectories.h>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include "localization_msgs/msg/pose_vel_acc_ang.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace ur5_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};
using LocationType = geometry_msgs::msg::PointStamped;

class UR5Controller : public controller_interface::ControllerInterface
{
public:
  UR5_CONTROLLER__VISIBILITY_PUBLIC
  UR5Controller();

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UR5_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  UR5_CONTROLLER__VISIBILITY_PUBLIC
  void setupMrt();
  UR5_CONTROLLER__VISIBILITY_PUBLIC
  void setupMpc();
  void topic_callback(const LocationType::SharedPtr msg) const;
  void tf_callback();

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ChassisCmdType = geometry_msgs::msg::Twist;

protected:
  std::shared_ptr<ur5_controller::ParamListener> param_listener_;
  ur5_controller::Params params_;
  void updateStateEstimation(const rclcpp::Time time,const rclcpp::Duration &period);

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<LocationType>::SharedPtr location_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<LocationType>> rt_location_ptr_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;
  // rclcpp::Publisher<geometry_msgs::msg::Twist> chassis_command_pub_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

  rclcpp::Subscription<localization_msgs::msg::PoseVelAccAng>::SharedPtr subscription_;

private:
  // callback for topic interface
  UR5_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  ocs2::TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const ocs2::SystemObservation& observation);
  std::string robot_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::mutex transform_mutex_;
  double x_, y_, yaw_;
  double roll_, pitch_;

  std::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  rclcpp::Node::SharedPtr mpc_node_;
  rclcpp::Node::SharedPtr mrt_node_;
  rclcpp::Node::SharedPtr target_node_;
  std::shared_ptr<ur5_interface::UR5Interface> ur5Interface_;
  std::shared_ptr<ocs2::MRT_ROS_Interface> mrtRosInterface_;
  std::shared_ptr<ocs2::MPC_ROS_Interface> mpcRosInterface_;
  std::shared_ptr<ur5_interface::UR5JoyTargetTrajectories> targetPoseCommandInterface_;
  ocs2::vector_t optimizedInput_;
  //State Estimation
  ocs2::SystemObservation currentObservation_;
  std::thread mpcThread_;
  std::thread mrtThread_;
  std::thread waitingThread_;
  std::thread targetThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  std::string task_file_;
  //use for debug
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obs_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_publisher_;
};

}  // namespace ur5_controller

#endif  // UR5_CONTROLLER__MMCO_CONTROLLER_HPP_
