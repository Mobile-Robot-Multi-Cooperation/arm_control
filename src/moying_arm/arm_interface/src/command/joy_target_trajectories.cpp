#include "arm_interface/command/joy_target_trajectories.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace arm_interface{
using namespace std::chrono_literals;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 订阅目标点 发布和更新轨迹
ARMJoyTargetTrajectories::ARMJoyTargetTrajectories(
    const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
    GaolPoseToTargetTrajectories gaolPoseToTargetTrajectories,std::string taskFile)
    : node_(node),
      gaolPoseToTargetTrajectories_(std::move(gaolPoseToTargetTrajectories)) {
  // observation subscriber
  // 回调函数 进行消息类型的转换
  auto observationCallback =
      [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
      };
  // 订阅消息 获取当前系统的状态（如关节位置、速度等）。
  observationSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
          topicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher 轨迹发布方
  targetTrajectoriesPublisherPtr_.reset(
      new TargetTrajectoriesRosPublisher(node_, topicPrefix));
  timer_ = node_->create_wall_timer(
      100ms, std::bind(&ARMJoyTargetTrajectories::timer_callback,this));
  // std::string joy_topic_name=node->get_parameter("joy_topic_name").as_string(); 订阅手柄话题
  joy_command_subsciption_ = node_->create_subscription<sensor_msgs::msg::Joy>("/arm/joy", rclcpp::SystemDefaultsQoS(), [this](const  sensor_msgs::msg::Joy::SharedPtr twist)
  {
    joy_command_ptr_.writeFromNonRT(twist);
  });
  goal_point_subscription_= node_->create_subscription<geometry_msgs::msg::Pose>("/contact1_pose", rclcpp::SystemDefaultsQoS(), [this](const  geometry_msgs::msg::Pose::SharedPtr gpoint)
  {
    goal_point_ptr_.writeFromNonRT(gpoint);
  });
  inc_goal_point_subscription_= node_->create_subscription<geometry_msgs::msg::Pose>("/del_contact1_pose", rclcpp::SystemDefaultsQoS(), [this](const  geometry_msgs::msg::Pose::SharedPtr fpoint)
  {
    inc_goal_point_ptr_.writeFromNonRT(fpoint);
  });
  // 把机械臂调到初始位置
  RCLCPP_INFO_STREAM(node_->get_logger(),"[ARMJoyTargetTrajectory] Loading...");
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    RCLCPP_INFO_STREAM(node_->get_logger(),"[ARMJoyTargetTrajectory] Loading task from:"<<taskFilePath);
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"[ARMJoyTargetTrajectory] Can't loading task file");
    throw std::invalid_argument("[MMCODualJoyTargetTrajectory] task file not found: " + taskFilePath.string());
  }
  ocs2::vector_t pos = ocs2::vector_t::Zero(3);
  ocs2::vector_t ori = ocs2::vector_t::Zero(4);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.position",pos);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.endEffector.orientation", ori);
  position_.x() = pos(0);
  position_.y() = pos(1);
  position_.z() = pos(2);
  arm_position_ = position_;
  // goal_position_.z() +=0.3;
  orientation_.w() = ori(0);
  orientation_.x() = ori(1);
  orientation_.y() = ori(2);
  orientation_.z() = ori(3);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  use_joy = false;

}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 回调函数，根据是否启用手柄生成轨迹并发布轨迹
void ARMJoyTargetTrajectories::timer_callback() {

  // get TargetTrajectories
  if(use_joy == false)
  {
    // auto joy_command = joy_command_ptr_.readFromRT();
    // if (!joy_command || !(*joy_command)) {
    //   RCLCPP_WARN_ONCE(node_->get_logger(),"[Joy] no pub yet.");
    //   return;
    // }
    RCLCPP_INFO_ONCE(node_->get_logger(),"[Joy] get.");
    auto gpoint = goal_point_ptr_.readFromRT();
    if (!gpoint|| !(*gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Goa Point] no pub yet.");
      return;
    }
    RCLCPP_INFO_ONCE(node_->get_logger(),"[GoalPoint] get.");
    // if((*joy_command)->axes[0] >0.8)
    // {
    //   use_joy= true;
    //   RCLCPP_INFO_ONCE(node_->get_logger(),"[Inc Goal Point] Inc mode....");
    //   return;
    // }
    position_.x()  =  (*gpoint)->position.x;
    position_.y()  =  (*gpoint)->position.y;
    position_.z()  =  (*gpoint)->position.z;
    Eigen::Quaterniond ori((*gpoint)->orientation.w,(*gpoint)->orientation.x,
      (*gpoint)->orientation.y,(*gpoint)->orientation.z);
    

    ocs2::SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }
    const auto targetTrajectories =
        gaolPoseToTargetTrajectories_(position_, orientation_*ori, observation);
    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        targetTrajectories);
  }
  else
  {
    auto joy_command = joy_command_ptr_.readFromRT();
    if (!joy_command || !(*joy_command)) {
      // std::cerr<<"No joy command received"<<std::endl;
      return;
    }
    auto inc_gpoint = inc_goal_point_ptr_.readFromRT();
    if (!inc_gpoint|| !(*inc_gpoint)) {
      RCLCPP_WARN_ONCE(node_->get_logger(),"[Inc Goal Point] no pub yet.");
      return;
    }
    RCLCPP_INFO_ONCE(node_->get_logger(),"[Inc Goal Point] Get Inc mode info.");
    auto now = node_->get_clock()->now();
    // Get the transform from 'world' to '/mor/right_inner_finger'
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("world", "arm/right_inner_finger", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
    position_.x() += (*inc_gpoint)->position.x;
    position_.y() += (*inc_gpoint)->position.y;
    position_.z() += (*inc_gpoint)->position.z;
    ocs2::SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }
    const auto targetTrajectories =
        gaolPoseToTargetTrajectories_(position_, orientation_, observation);
    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        targetTrajectories);
  }


}
}  // namespace mmco_dual_interfac