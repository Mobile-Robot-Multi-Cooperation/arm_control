#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ur5_interface/ur5_module_info.h"
#include "ur5_interface/ur5_precomputation.h"
#include "ur5_interface/ur5_pinocchio_mapping.h"

namespace ur5_interface{

/** Callback for caching and reference update */
class UR5PreComputation : public PreComputation {
 public:
  UR5PreComputation(ocs2::PinocchioInterface pinocchioInterface, const UR5ModuleInfo& info);

  ~UR5PreComputation() override = default;

  UR5PreComputation(const UR5PreComputation& rhs) = delete; // 禁用拷贝构造
  UR5PreComputation* clone() const override;

  void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
  void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

  ocs2::PinocchioInterface& getPinocchioInterface() {return pinocchioInterface_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  UR5PinocchioMapping pinocchioMapping_;
};

}  // namespace mobile_manipulator