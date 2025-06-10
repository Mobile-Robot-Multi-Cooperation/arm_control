#include "arm_interface/arm_pinocchio_mapping.h"
#include "arm_interface/arm_module_info.h" 

// 将ocs2数据映射到pinocchio
namespace arm_interface{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
ARMPinocchioMappingTpl<SCALAR>::ARMPinocchioMappingTpl(ARMModuleInfo info)
    : moduleInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 拷贝构造创建新对象
template <typename SCALAR>
ARMPinocchioMappingTpl<SCALAR>* ARMPinocchioMappingTpl<SCALAR>::clone() const {
  return new ARMPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 返回系统状态向量（位置）
template <typename SCALAR>
auto ARMPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto ARMPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {

  vector_t vPinocchio = vector_t::Zero(moduleInfo_.stateDim);
  vPinocchio = input;

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto ARMPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  return {Jq, Jv};
}

// explicit template instantiation
template class arm_interface::ARMPinocchioMappingTpl<ocs2::scalar_t>;
template class arm_interface::ARMPinocchioMappingTpl<ocs2::ad_scalar_t>;
} // namespace