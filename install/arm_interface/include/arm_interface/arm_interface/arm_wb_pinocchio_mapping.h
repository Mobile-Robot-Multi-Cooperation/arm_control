#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "arm_interface/arm_module_info.h"
namespace arm_interface{
using namespace ocs2;
template<typename SCALAR>
class ARMWBPinocchioMappingTpl;

using ARMWBPinocchioMapping = ARMWBPinocchioMappingTpl<ocs2::scalar_t>;
using ARMWBPinocchioMappingCppAd = ARMWBPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class ARMWBPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit ARMWBPinocchioMappingTpl(ARMModuleInfo info);

  ~ARMWBPinocchioMappingTpl() override = default;
  ARMWBPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const ARMModuleInfo& getARMModuleInfo() const { return moduleInfo_; }
private:
  ARMWBPinocchioMappingTpl(const ARMWBPinocchioMappingTpl& rhs) = default;

  const ARMModuleInfo moduleInfo_;
};

}