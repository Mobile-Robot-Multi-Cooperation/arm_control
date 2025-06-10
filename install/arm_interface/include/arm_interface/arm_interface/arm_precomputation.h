#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "arm_interface/arm_module_info.h"
#include "arm_interface/arm_precomputation.h"
#include "arm_interface/arm_pinocchio_mapping.h"

namespace arm_interface{

/** Callback for caching and reference update */
class ARMPreComputation : public PreComputation {
 public:
  ARMPreComputation(ocs2::PinocchioInterface pinocchioInterface, const ARMModuleInfo& info);

  ~ARMPreComputation() override = default;

  ARMPreComputation(const ARMPreComputation& rhs) = delete; // 禁用拷贝构造
  ARMPreComputation* clone() const override;

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
  void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

  ocs2::PinocchioInterface& getPinocchioInterface() {return pinocchioInterface_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  ARMPinocchioMapping pinocchioMapping_;
};

}  // namespace mobile_manipulator