#include "ur5_controller/ur5_controller.hpp"

#include <limits>
#include <memory>
#include <math.h>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <command/joy_target_trajectories.h>
#include <command/ur5_target_trajectories_interactive_marker.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{  // utility 话题
  constexpr auto DEFAULT_LOCATION_TOPIC="/location";
// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

//   using ControllerReferenceMsg = control_msgs::msg::JointJog; 控制消息
using ControllerReferenceMsg = ur5_controller::UR5Controller::ControllerReferenceMsg;

// called from RT control loop 重置参考消息
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace ur5_controller
{
UR5Controller::UR5Controller() : controller_interface::ControllerInterface(), rt_location_ptr_(nullptr){}

controller_interface::CallbackReturn UR5Controller::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);


  location_sub_ = get_node()->create_subscription<LocationType>(DEFAULT_LOCATION_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const LocationType::SharedPtr msg)
  {
      rt_location_ptr_.writeFromNonRT(msg);
  });
  try
  {
    param_listener_ = std::make_shared<ur5_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // 获取文件路径 从yaml文件中获取名称
  std::string task_package_name = param_listener_->get_params().task_package_name;
  std::string urdf_package_name = param_listener_->get_params().urdf_package_name;
  std::string lib_package_name = param_listener_->get_params().lib_package_name;

  std::string task_path =  ament_index_cpp::get_package_share_directory(task_package_name);
  std::string urdf_path =  ament_index_cpp::get_package_share_directory(urdf_package_name);
  std::string lib_path =  ament_index_cpp::get_package_share_directory(lib_package_name);

  if(task_path.length() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get task package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(urdf_path.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get urdf package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(lib_path.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get lib package path");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string urdfFile;
  std::string taskFile;
  std::string libFold;

  urdfFile = urdf_path + param_listener_->get_params().urdf_file;
  taskFile = task_path + param_listener_->get_params().task_file;
  task_file_ = taskFile;
  libFold =  lib_path + param_listener_->get_params().lib_folder;

  // cmd_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Twist>("/mor/mecanum_controller/cmd_vel", 10);
  robot_name_ = param_listener_->get_params().robot_name;
  // ocs2接口定义
  ur5Interface_ = std::make_shared<ur5_interface::UR5Interface>(taskFile, libFold, urdfFile);
  // 三个节点
  mpc_node_ = rclcpp::Node::make_shared(
    robot_name_ + "_mpc",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));
  mrt_node_ = rclcpp::Node::make_shared(
    robot_name_+ "_mrt",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));
  target_node_ = rclcpp::Node::make_shared(
    robot_name_+ "_traget",
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true));

  // 初始化Mpc和Mrt
  setupMpc();
  setupMrt();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  update_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
          this->tf_callback();
      });
  //use for debug
  cmd_publisher_ =  get_node()->create_publisher<geometry_msgs::msg::Twist>("ur5/cmd_vel", 10);
  ref_publisher_ =  get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mpc_cmd", 10);
  obs_publisher_ =  get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mpc_obs", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

// 意义不大，固定的，可以改成末端的tf
void UR5Controller::tf_callback() {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        try {
            auto transform_stamped = tf_buffer_->lookupTransform("world", "base_link", tf2::TimePointZero);
            x_ = transform_stamped.transform.translation.x;
            y_ = transform_stamped.transform.translation.y;
            tf2::Quaternion quat(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            tf2::Matrix3x3 mat(quat);
            mat.getRPY(roll_, pitch_, yaw_);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_node()->get_logger(), "Could not transform 'world' to '/ur5/base_link': %s", ex.what());
        }
}

void UR5Controller::setupMpc()
{
  mpc_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(ur5Interface_->mpcSettings(),ur5Interface_->ddpSettings(),ur5Interface_->getRollout(),
                                            ur5Interface_->getOptimalControlProblem(),ur5Interface_->getInitializer());
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robot_name_, ur5Interface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(mpc_node_);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpcRosInterface_ = std::make_shared<ocs2::MPC_ROS_Interface>(*mpc_,robot_name_);
}

void UR5Controller::setupMrt()
{
  mrtRosInterface_ = std::make_shared<ocs2::MRT_ROS_Interface>(robot_name_);
  mrtRosInterface_->initRollout(&(ur5Interface_->getRollout()));
}

controller_interface::CallbackReturn UR5Controller::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&UR5Controller::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);


  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void UR5Controller::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), params_.joints.size());
  }
}

// 对应hardware_interface的command_interface
controller_interface::InterfaceConfiguration UR5Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size() );
  for (const auto & joint : params_.joints)
  {
    if(params_.use_chain_interface == false)
      command_interfaces_config.names.push_back(joint + "/" + "velocity");
      // command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    else
      command_interfaces_config.names.push_back(params_.chain_velocity_interface_name+"/"+joint + "/" + "velocity");
      // command_interfaces_config.names.push_back(params_.chain_velocity_interface_name+"/"+joint + "/" + hardware_interface::HW_IF_VELOCITY);

  }

  return command_interfaces_config;
}

// 对应hardware_interface的state_interface
controller_interface::InterfaceConfiguration UR5Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(state_joints_.size() );
  for (const auto & joint : state_joints_)
  {
    state_interfaces_config.names.push_back(joint + "/" + "position");
    // state_interfaces_config.names.push_back(joint + "/" + "velocity");
    // state_interfaces_config.names.push_back(joint + "/" + "effort");
    // state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    // state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return state_interfaces_config;
}

ocs2::TargetTrajectories UR5Controller::goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const ocs2::SystemObservation& observation)
{
  const ocs2::scalar_array_t timeTrajectory{currentObservation_.time};
  // state trajectory: 3 + 4 for des/ired position vector and orientation quaternion
  Eigen::Vector3d rpos = position;
  
  const ocs2::vector_t target = (ocs2::vector_t(7) << rpos, orientation.coeffs()).finished();
  RCLCPP_INFO(get_node()->get_logger(),"target:%f,%f,%f",position[0],position[1],position[2]);
  // 状态轨迹
  const ocs2::vector_array_t stateTrajectory{target};
  // input trajectory 输入轨迹
  const ocs2::vector_array_t inputTrajectory{ocs2::vector_t::Zero(observation.input.size())};
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

controller_interface::CallbackReturn UR5Controller::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  RCLCPP_INFO(get_node()->get_logger(),"on activate");
  // Set default value in command
  bool use_gui_target = param_listener_->get_params().use_gui_target;
  if(use_gui_target)
  {
    auto bound_function = std::bind(&UR5Controller::goalPoseToTargetTrajectories, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
    targetPoseCommandInterface_ = std::make_shared<ur5_interface::UR5JoyTargetTrajectories>(target_node_,robot_name_,bound_function,task_file_);
    targetThread_ = std::thread([&](){
      targetPoseCommandInterface_->publishInteractiveMarker();
    });
  }
  // 启动MPC
  mpcThread_ = std::thread([&](){
    mpcRosInterface_->launchNodes(mpc_node_);
  });
  ocs2::setThreadPriority(2,mpcThread_);
  mrtRosInterface_->launchNodes(mrt_node_);
  ocs2::vector_t state(6);
  for(size_t i=0;i<6;i++)
    state(i)=0.0;
  currentObservation_.state = state;
    command_interfaces_[0].set_value(0);
    command_interfaces_[1].set_value(0);
    command_interfaces_[2].set_value(0);
    command_interfaces_[3].set_value(0);
    command_interfaces_[4].set_value(0);
    command_interfaces_[5].set_value(0);

  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UR5Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  if (targetThread_.joinable()) {
    targetThread_.join();
  }
  if (waitingThread_.joinable()) {
    waitingThread_.join();
  }
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// 获取机械臂的状态
void UR5Controller::updateStateEstimation(const rclcpp::Time time,const rclcpp::Duration &period)
{
  std::lock_guard<std::mutex> lock(transform_mutex_);

  currentObservation_.time += period.seconds();
  ocs2::vector_t state(6);

  state(0) = state_interfaces_[0].get_value();
  state(1) = state_interfaces_[1].get_value();
  state(2) = state_interfaces_[2].get_value();
  state(3) = state_interfaces_[3].get_value();
  state(4) = state_interfaces_[4].get_value();
  state(5) = state_interfaces_[5].get_value();
  currentObservation_.state = state;
}

// 更新控制器的状态和执行控制逻辑,启动后会一直运行
controller_interface::return_type UR5Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  updateStateEstimation(time,period);
  static bool first_restart = false;
  static bool mpc_node_well = false;
  // 刚开始从task加载末端执行器的位置
  if(first_restart == false)
  {
    ocs2::SystemObservation initObservation;
    initObservation = currentObservation_;
    ocs2::vector_t initTarget(7);
    ocs2::vector_t pos = ocs2::vector_t::Zero(3);
    ocs2::vector_t ori = ocs2::vector_t::Zero(4);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.position",pos);
    ocs2::loadData::loadEigenMatrix(task_file_, "initialState.endEffector.orientation", ori);
    Eigen::Quaterniond eig_ori(ori(0),ori(1),ori(2),ori(3));


    initTarget.head(3) << pos;
    initTarget.tail(4) << eig_ori.coeffs();
    const ocs2::vector_t zeroInput =
        ocs2::vector_t::Zero(ur5Interface_->getUR5ModuelInfo().inputDim);
    const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time},
                                                    {initTarget}, {zeroInput});
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been receivedzz.");
    mrtRosInterface_->resetMpcNode(initTargetTrajectories);
    mrtRosInterface_->setCurrentObservation(initObservation);
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been received.");
    first_restart = true;
  }
  if(mpc_node_well==false){
    RCLCPP_INFO_STREAM(get_node()->get_logger(), 
    "Checking: initialPolicyReceived=" << mrtRosInterface_->initialPolicyReceived() 
      << ", rclcpp::ok()=" << rclcpp::ok() << robot_name_);
    if(!mrtRosInterface_->initialPolicyReceived() && rclcpp::ok()){
      mrtRosInterface_->spinMRT();
      RCLCPP_INFO_STREAM(get_node()->get_logger(), 
      "After spinMRT(), initialPolicyReceived=" << mrtRosInterface_->initialPolicyReceived());
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Policy received after waiting.");
      mrtRosInterface_->setCurrentObservation(currentObservation_);
    }
    else
    {
      mpc_node_well = true;
    }
    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    command_interfaces_[2].set_value(0.0);
    command_interfaces_[3].set_value(0.0);
    command_interfaces_[4].set_value(0.0);
    command_interfaces_[5].set_value(0.0);
    return controller_interface::return_type::OK;
  }
  mrtRosInterface_->setCurrentObservation(currentObservation_);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been receivedrr.");
  mrtRosInterface_->spinMRT();
  mrtRosInterface_->updatePolicy();
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  const ocs2::scalar_t dt = 1.0/ur5Interface_->mpcSettings().mrtDesiredFrequency_;
  mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state,optimizedState, optimizedInput, plannedMode);
  // mrtRosInterface_->rolloutPolicy(currentObservation_.time, currentObservation_.state,dt,optimizedState, optimizedInput, plannedMode);
  currentObservation_.input = optimizedInput;
  command_interfaces_[0].set_value(optimizedInput(0));
  command_interfaces_[1].set_value(optimizedInput(1));
  command_interfaces_[2].set_value(optimizedInput(2));
  command_interfaces_[3].set_value(optimizedInput(3));
  command_interfaces_[4].set_value(optimizedInput(4));
  command_interfaces_[5].set_value(optimizedInput(5));

  return controller_interface::return_type::OK;
}

}  // namespace ur5_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur5_controller::UR5Controller, controller_interface::ControllerInterface)