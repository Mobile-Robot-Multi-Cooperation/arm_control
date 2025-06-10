#ifndef ARM_CONTROLLER__VALIDATE_ARM_CONTROLLER_PARAMETERS_HPP_
#define ARM_CONTROLLER__VALIDATE_ARM_CONTROLLER_PARAMETERS_HPP_

#include <string>

#include "parameter_traits/parameter_traits.hpp"

namespace parameter_traits
{
Result forbidden_interface_name_prefix(rclcpp::Parameter const & parameter)
{
  auto const & interface_name = parameter.as_string();

  if (interface_name.rfind("blup_", 0) == 0)
  {
    return ERROR("'interface_name' parameter can not start with 'blup_'");
  }
  
  return OK;
}
}  // namespace parameter_traits

#endif  // ARM_CONTROLLER__VALIDATE_ARM_CONTROLLER_PARAMETERS_HPP_