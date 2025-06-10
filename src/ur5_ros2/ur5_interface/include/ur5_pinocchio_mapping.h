#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "ur5_interface/ur5_module_info.h"
namespace ur5_interface{
using namespace ocs2;
template<typename SCALAR>
class UR5PinocchioMappingTpl;

using UR5PinocchioMapping = UR5PinocchioMappingTpl<ocs2::scalar_t>;
using UR5PinocchioMappingCppAd = UR5PinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class UR5PinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit UR5PinocchioMappingTpl(UR5ModuleInfo info);

  ~UR5PinocchioMappingTpl() override = default;
  UR5PinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const UR5ModuleInfo& getUR5ModuleInfo() const { return moduleInfo_; }
private:
  UR5PinocchioMappingTpl(const UR5PinocchioMappingTpl& rhs) = default;

  const UR5ModuleInfo moduleInfo_;
};

}