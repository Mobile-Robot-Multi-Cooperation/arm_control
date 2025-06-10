#pragma once

#include <memory>

#include <arm_interface/arm_precomputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace arm_interface{
using namespace ocs2;
class ARMSelfCollisionConstraint final : public SelfCollisionConstraint {
public:
  ARMSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~ARMSelfCollisionConstraint() override = default;
  ARMSelfCollisionConstraint(const ARMSelfCollisionConstraint& other) = default;
  ARMSelfCollisionConstraint* clone() const { return new ARMSelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<ARMPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace mobile_manipulator