#include "ur5_interface/ur5_pinocchio_mapping.h"
#include "ur5_interface/ur5_module_info.h" 

// 将ocs2数据映射到pinocchio
namespace ur5_interface{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
UR5PinocchioMappingTpl<SCALAR>::UR5PinocchioMappingTpl(UR5ModuleInfo info)
    : moduleInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 拷贝构造创建新对象
template <typename SCALAR>
UR5PinocchioMappingTpl<SCALAR>* UR5PinocchioMappingTpl<SCALAR>::clone() const {
  return new UR5PinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 返回系统状态向量（位置）
template <typename SCALAR>
auto UR5PinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto UR5PinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {

  vector_t vPinocchio = vector_t::Zero(moduleInfo_.stateDim);
  vPinocchio = input;

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto UR5PinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  return {Jq, Jv};
}

// explicit template instantiation
template class ur5_interface::UR5PinocchioMappingTpl<ocs2::scalar_t>;
template class ur5_interface::UR5PinocchioMappingTpl<ocs2::ad_scalar_t>;
} // namespace