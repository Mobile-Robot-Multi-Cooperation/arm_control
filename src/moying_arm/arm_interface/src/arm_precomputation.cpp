#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "arm_interface/arm_precomputation.h"
#include "arm_interface/arm_module_info.h"

namespace arm_interface{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ARMPreComputation::ARMPreComputation(PinocchioInterface pinocchioInterface, const ARMModuleInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ARMPreComputation* ARMPreComputation::clone() const {
  return new ARMPreComputation(pinocchioInterface_, pinocchioMapping_.getARMModuleInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// request预计算，输入需求（如成本、约束、雅可比矩阵等），时间点，状态和输入向量
void ARMPreComputation::request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) {
  if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(ocs2::Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateGlobalPlacements(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ARMPreComputation::requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) {
  if (!request.containsAny(ocs2::Request::Cost + ocs2::Request::Constraint + ocs2::Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);

  if (request.contains(ocs2::Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace mmco_dual_interface