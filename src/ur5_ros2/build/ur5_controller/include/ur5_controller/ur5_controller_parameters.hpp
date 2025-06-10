// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>

#include "validate_ur5_controller_parameters.hpp"


namespace ur5_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::vector<std::string> joints = {};
        std::vector<std::string> state_joints = {};
        std::string interface_name = "";
        std::string urdfFile = "";
        std::string taskFile = "";
        std::string libFile = "";
        std::string robot_name = "ur5";
        bool use_gui_target = true;
        bool use_effort_interface = true;
        std::string task_package_name = "ur5_interface";
        std::string task_file = "config/ur5/task.info";
        std::string urdf_package_name = "ur5_controller";
        std::string urdf_file = "ur5_description";
        std::string lib_package_name = "ur5_controller";
        std::string lib_folder = "auto_generated";
        bool use_chain_interface = false;
        std::string chain_velocity_interface_name = "velocity_interface_name";
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        bool use_gui_target = true;
        bool use_effort_interface = true;
        bool use_chain_interface = false;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    ParamListener(rclcpp::Node::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("ur5_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "") {
      logger_ = std::move(logger);
      prefix_ = prefix;
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.use_gui_target = params.use_gui_target;
      output.use_effort_interface = params.use_effort_interface;
      output.use_chain_interface = params.use_chain_interface;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "joints")) {
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "state_joints")) {
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.state_joints = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "interface_name")) {
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = one_of<std::string>(param, {"position", "velocity", "acceleration", "effort"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = forbidden_interface_name_prefix(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.interface_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "urdfFile")) {
            updated_params.urdfFile = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "taskFile")) {
            updated_params.taskFile = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "libFile")) {
            updated_params.libFile = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "robot_name")) {
            updated_params.robot_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_gui_target")) {
            updated_params.use_gui_target = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_effort_interface")) {
            updated_params.use_effort_interface = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "task_package_name")) {
            updated_params.task_package_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "task_file")) {
            updated_params.task_file = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "urdf_package_name")) {
            updated_params.urdf_package_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "urdf_file")) {
            updated_params.urdf_file = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "lib_package_name")) {
            updated_params.lib_package_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "lib_folder")) {
            updated_params.lib_folder = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_chain_interface")) {
            updated_params.use_chain_interface = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "chain_velocity_interface_name")) {
            updated_params.chain_velocity_interface_name = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Specifies joints used by the controller. If state joints parameter is defined, then only command joints are defined with this parameter.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.joints);
          parameters_interface_->declare_parameter(prefix_ + "joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "state_joints")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "(optional) Specifies joints for reading states. This parameter is only relevant when state joints are different then command joint, i.e., when a following controller is used.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.state_joints);
          parameters_interface_->declare_parameter(prefix_ + "state_joints", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "interface_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the interface used by the controller on joints and command_joints.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.interface_name);
          parameters_interface_->declare_parameter(prefix_ + "interface_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "urdfFile")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "URDF file used for the controller.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.urdfFile);
          parameters_interface_->declare_parameter(prefix_ + "urdfFile", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "taskFile")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the task file used for the controller.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.taskFile);
          parameters_interface_->declare_parameter(prefix_ + "taskFile", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "libFile")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the library file used for the controller.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.libFile);
          parameters_interface_->declare_parameter(prefix_ + "libFile", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "robot_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of robot.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.robot_name);
          parameters_interface_->declare_parameter(prefix_ + "robot_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_gui_target")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Rviz Marker Target.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.use_gui_target);
          parameters_interface_->declare_parameter(prefix_ + "use_gui_target", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_effort_interface")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "real robot use effot interface.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.use_effort_interface);
          parameters_interface_->declare_parameter(prefix_ + "use_effort_interface", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "task_package_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for task.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.task_package_name);
          parameters_interface_->declare_parameter(prefix_ + "task_package_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "task_file")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the task file used for the controller.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.task_file);
          parameters_interface_->declare_parameter(prefix_ + "task_file", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "urdf_package_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for URDF.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.urdf_package_name);
          parameters_interface_->declare_parameter(prefix_ + "urdf_package_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "urdf_file")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for URDF.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.urdf_file);
          parameters_interface_->declare_parameter(prefix_ + "urdf_file", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "lib_package_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for library.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.lib_package_name);
          parameters_interface_->declare_parameter(prefix_ + "lib_package_name", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "lib_folder")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for library.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.lib_folder);
          parameters_interface_->declare_parameter(prefix_ + "lib_folder", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_chain_interface")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for library.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.use_chain_interface);
          parameters_interface_->declare_parameter(prefix_ + "use_chain_interface", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "chain_velocity_interface_name")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Name of the package used for library.";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.chain_velocity_interface_name);
          parameters_interface_->declare_parameter(prefix_ + "chain_velocity_interface_name", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'joints': {}", validation_result.error()));
      }
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'joints': {}", validation_result.error()));
      }
      updated_params.joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "state_joints");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'state_joints': {}", validation_result.error()));
      }
      updated_params.state_joints = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "interface_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'interface_name': {}", validation_result.error()));
      }
      if(auto validation_result = one_of<std::string>(param, {"position", "velocity", "acceleration", "effort"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'interface_name': {}", validation_result.error()));
      }
      if(auto validation_result = forbidden_interface_name_prefix(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'interface_name': {}", validation_result.error()));
      }
      updated_params.interface_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "urdfFile");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.urdfFile = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "taskFile");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.taskFile = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "libFile");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.libFile = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "robot_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.robot_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "use_gui_target");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_gui_target = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "use_effort_interface");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_effort_interface = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "task_package_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.task_package_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "task_file");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.task_file = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "urdf_package_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.urdf_package_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "urdf_file");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.urdf_file = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "lib_package_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.lib_package_name = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "lib_folder");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.lib_folder = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "use_chain_interface");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_chain_interface = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "chain_velocity_interface_name");
      RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.chain_velocity_interface_name = param.as_string();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;

      // rclcpp::Logger cannot be default-constructed
      // so we must provide a initialization here even though
      // every one of our constructors initializes logger_
      rclcpp::Logger logger_ = rclcpp::get_logger("ur5_controller");
      std::mutex mutable mutex_;
  };

} // namespace ur5_controller
