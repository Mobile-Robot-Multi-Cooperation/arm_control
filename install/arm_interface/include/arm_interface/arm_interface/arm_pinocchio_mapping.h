#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "arm_interface/arm_module_info.h"
namespace arm_interface{
using namespace ocs2;
template<typename SCALAR>
class ARMPinocchioMappingTpl;

using ARMPinocchioMapping = ARMPinocchioMappingTpl<ocs2::scalar_t>;
using ARMPinocchioMappingCppAd = ARMPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class ARMPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR>{
public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;
  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit ARMPinocchioMappingTpl(ARMModuleInfo info);

  ~ARMPinocchioMappingTpl() override = default;
  ARMPinocchioMappingTpl<SCALAR>* clone() const override;
  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  const ARMModuleInfo& getARMModuleInfo() const { return moduleInfo_; }
private:
  ARMPinocchioMappingTpl(const ARMPinocchioMappingTpl& rhs) = default;

  const ARMModuleInfo moduleInfo_;
};

}