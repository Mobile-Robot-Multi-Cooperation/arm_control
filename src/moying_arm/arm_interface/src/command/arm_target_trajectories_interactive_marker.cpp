#include "arm_interface/command/arm_target_trajectories_interactive_marker.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

namespace arm_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 通过 RViz 等工具直观地设置机械臂的目标轨迹
ARMTargetTrajectoriesInteractiveMarker::ARMTargetTrajectoriesInteractiveMarker(
    const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
    GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories)
    : node_(node),
      arm_server_("arm_simple_marker", node_),
      gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)) {
  // observation subscriber
  auto observationCallback =
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
      };
  observationSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
          topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(
      new TargetTrajectoriesRosPublisher(node_, topicPrefix));

  // create an interactive marker for our server
  auto arm_feedback_cb =
      [&](const visualization_msgs::msg::InteractiveMarkerFeedback::
              ConstSharedPtr& feedback) {armProcessFeedback(feedback); };
  arm_menuHandler_.insert("Send arm target pose", arm_feedback_cb);

  // create an interactive marker for our server

  position_.resize(1);
  position_[0](0) = 0;
  position_[0](1) = -0.5;
  position_[0](2) = 0.9;
  orientation_.resize(1);
  orientation_[0] = Eigen::Quaterniond::Identity();
  auto armInteractiveMarker = createInteractiveMarker(0);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  arm_server_.insert(
      armInteractiveMarker);  //,
                           // boost::bind(&MMCODualTargetTrajectoriesInteractiveMarker::processFeedback,
                           // this, _1));
  arm_menuHandler_.apply(arm_server_, armInteractiveMarker.name);

  // 'commit' changes and send to all clients
  arm_server_.applyChanges();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 调整机械臂的姿态
visualization_msgs::msg::InteractiveMarker
ARMTargetTrajectoriesInteractiveMarker::createInteractiveMarker(int mode) const {
  visualization_msgs::msg::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "world";
  interactiveMarker.header.stamp = node_->now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.3;
  interactiveMarker.description = "ClickMe";
  interactiveMarker.pose.position.x = position_[mode](0);
  interactiveMarker.pose.position.y = position_[mode](1);
  interactiveMarker.pose.position.z = position_[mode](2);
  interactiveMarker.pose.orientation.w = orientation_[mode].w();
  interactiveMarker.pose.orientation.x = orientation_[mode].x();
  interactiveMarker.pose.orientation.y = orientation_[mode].y();
  interactiveMarker.pose.orientation.z = orientation_[mode].z();


  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::msg::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 根据交互操作生成对应的轨迹
void ARMTargetTrajectoriesInteractiveMarker::armProcessFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
        feedback) {
  // Desired state trajectory
  const Eigen::Vector3d position(feedback->pose.position.x,
                                 feedback->pose.position.y,
                                 feedback->pose.position.z);
  const Eigen::Quaterniond orientation(
      feedback->pose.orientation.w, feedback->pose.orientation.x,
      feedback->pose.orientation.y, feedback->pose.orientation.z);
  position_[0] = position;
  orientation_[0] = orientation;

  // get the latest observation
  ocs2::SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    observation = latestObservation_;
  }

  // get TargetTrajectories
  const auto targetTrajectories =
      gaolPoseToTargetTrajectories_(position_, orientation_, observation);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
      targetTrajectories);
}

}  // namespace mmco_dual_interface