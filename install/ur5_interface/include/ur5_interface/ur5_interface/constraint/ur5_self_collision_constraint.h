#pragma once

#include <memory>

#include <ur5_interface/ur5_precomputation.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

namespace ur5_interface{
using namespace ocs2;
class UR5SelfCollisionConstraint final : public SelfCollisionConstraint {
public:
  UR5SelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~UR5SelfCollisionConstraint() override = default;
  UR5SelfCollisionConstraint(const UR5SelfCollisionConstraint& other) = default;
  UR5SelfCollisionConstraint* clone() const { return new UR5SelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<UR5PreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace mobile_manipulator