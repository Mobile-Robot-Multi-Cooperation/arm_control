#include "ur5_interface/dynamics/ur5_dynamics.h"

namespace ur5_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 计算移动机械臂的底盘动力学
UR5Dynamics::UR5Dynamics(const UR5ModuleInfo& info, const std::string& modelName,
                                                                         const std::string& modelFolder,
                                                                         bool recompileLibraries, bool verbose)
    : info_(std::move(info)) {
  this->initialize(info_.stateDim, info_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t UR5Dynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                               const ad_vector_t&) const {
  // ad_vector_t dxdt(info_.stateDim);
  // dxdt = input;
  return input;
}

}  // namespace mobile_manipulator