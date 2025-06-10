#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ur5_interface/ur5_module_info.h"

namespace ur5_interface{
using namespace ocs2;

/**
 * Implementation of a wheel-based mobile manipulator.
 *
 * The wheel-based manipulator is simulated 2D-bicycle model for the base. The state
 * of the robot is: (base x, base y, base yaw, arm joints).
 *
 * The robot is assumed to be velocity controlled with the base commands as the forward
 * velocity and the angular velocity around z.
 */
class UR5Dynamics final : public SystemDynamicsBaseAD {
 public:
  /**
   * Constructor
   *
   * @param [in] modelInfo : The manipulator information.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose : Display information.
   */
  UR5Dynamics(const UR5ModuleInfo& modelInfo, const std::string& modelName,
                                      const std::string& modelFolder, bool recompileLibraries = true, bool verbose = true);

  ~UR5Dynamics() override = default;
  UR5Dynamics* clone() const override { return new UR5Dynamics(*this); }

 private:
  UR5Dynamics(const UR5Dynamics& rhs) = default;

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;

  const UR5ModuleInfo info_;
};

}  // namespace mobile_manipulator