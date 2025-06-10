
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include "ocs2_mobile_manipulator/dynamics/DefaultManipulatorDynamics.h"

#include "arm_interface/arm_interface.h"

#include "arm_interface/constraint/end_effort_constraint.h"
// #include "arm_interface/constraint/base_constraint.h"
#include "arm_interface/constraint/arm_self_collision_constraint.h"
#include "arm_interface/dynamics/arm_dynamics.h"
#include "arm_interface/factory_functions.h"
#include "arm_interface/arm_precomputation.h"
#include "arm_interface/arm_pinocchio_mapping.h"
#include "arm_interface/cost/quadratic_input_cost.h"
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


namespace arm_interface {
using namespace ocs2;
// 构造函数，从配置文件中获取数据，加载各种参数，创建模型
ARMInterface::ARMInterface(const std::string& taskFile, const std::string& libraryFolder,
                                      const std::string& urdfFile)
{
  // 检查任务文件是否存在
  boost::filesystem::path taskFilePath(taskFile); 
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[armInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[armInterface] Task file not found: " + taskFilePath.string());
  }
  // 检查urdf文件是否存在
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[armInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[armInterface] URDF file not found: " + urdfFilePath.string());
  }
  // 检查库文件是否存在
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[armInterface] Generated library path: " << libraryFolderPath << std::endl;


  // 使用属性数存储数据（XML、JSON、INFO 等格式的文件）
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  // 从配置文件（树）获取数据
  std::vector<std::string> removeJointNames;
  ocs2::loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
  std::string baseFrame, eeFrame;
  ocs2::loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
  ocs2::loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### Model: arm ";
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";
  std::cerr << "\n #### model_information.eeFrame: \"" << eeFrame << "\"" ;
  std::cerr << " #### =============================================================================" << std::endl;
  
  //  初始化Pinocchio接口 用来进行动力学和运动学计算
  std::cerr << " #### =============================================================================1" << std::endl;
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createPinocchioInterface(urdfFile,removeJointNames))); // 把rmj改为fixed
  std::cerr << " #### =============================================================================" << std::endl;
  std::cerr << *pinocchioInterfacePtr_;

  // 创建 armModuleInfo 对象
  armModuleInfo_ = arm_interface::createARMModuleInfo(*pinocchioInterfacePtr_,baseFrame, eeFrame);

  // 从配置树读取参数
  bool usePreComputation = true; // 是否使用预计算功能
  bool recompileLibraries = true; // 是否重新编译库
  std::cerr << "\n #### Model Settings:";
  std::cerr << "\n #### =============================================================================\n";
  ocs2::loadData::loadPtreeValue(pt, usePreComputation, "model_settings.usePreComputation", true);
  ocs2::loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
  std::cerr << " #### =============================================================================\n";

  // 初始化并从task加载数据 机器人的维度    修改taskinfo
  initialState_.setZero(armModuleInfo_.stateDim);
  const int baseStateDim = armModuleInfo_.stateDim - armModuleInfo_.armDim;
  const int armStateDim = armModuleInfo_.armDim;
  
  if (baseStateDim > 0) {
    vector_t initialBaseState = vector_t::Zero(baseStateDim);
    loadData::loadEigenMatrix(taskFile, "initialState.base.defaultManipulator" , initialBaseState);
    initialState_.head(baseStateDim) = initialBaseState;
  }

  vector_t initialArmState = vector_t::Zero(armStateDim);
  loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
  initialState_.tail(armStateDim) = initialArmState;

  std::cerr << "Initial State:   " << initialState_.transpose() << std::endl;

  ddpSettings_ = ocs2::ddp::loadSettings(taskFile,"ddp"); // 加载DDP设置
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile,"mpc"); // 加载MPC设置

  // 管理参考轨迹 std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;
  referenceManagerPtr_.reset(new ocs2::ReferenceManager);

  // Cost 成本 ocs2::OptimalControlProblem problem_;
  problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

  // Constraints
  // joint limits constraint 关节软约束 没定义？？？
  problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(*pinocchioInterfacePtr_, taskFile));
  // end-effector state constraint 末端执行器约束
  //Arm 末端约束和最终状态约束 没定义？？？
  problem_.stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector",
                                                                                usePreComputation, libraryFolder, recompileLibraries, armModuleInfo_.eeFrame));
  problem_.finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector",
                                                                                    usePreComputation, libraryFolder, recompileLibraries, armModuleInfo_.eeFrame));
  // 自碰撞约束
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  if (activateSelfCollision) {
    problem_.stateSoftConstraintPtr->add(
        "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, urdfFile, "selfCollision", usePreComputation,
                                                    libraryFolder, recompileLibraries));
  }
  // 动力学模型??????
  problem_.dynamicsPtr.reset(
          new ARMDynamics(armModuleInfo_, "dynamics", libraryFolder, recompileLibraries, true));

  // 预计算，只是一个指针
  if (usePreComputation) {
    problem_.preComputationPtr.reset(new ARMPreComputation(*pinocchioInterfacePtr_, armModuleInfo_));
  }

  // Rollout 滚动时域
  const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));
  std::cerr << " #### =============================================================================" << std::endl;

  // Initialization 给最优化问题提供初始解
  initializerPtr_.reset(new DefaultInitializer(armModuleInfo_.inputDim));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 从task获取输入成本矩阵 R
std::unique_ptr<ocs2::StateInputCost> ARMInterface::getQuadraticInputCost(const std::string& taskFile)
{
  ocs2::matrix_t R = ocs2::matrix_t::Zero(armModuleInfo_.inputDim, armModuleInfo_.inputDim); // 零矩阵
  const int baseInputDim = armModuleInfo_.inputDim - armModuleInfo_.armDim;
  const int armStateDim = armModuleInfo_.armDim ;


  // arm base DOFs input costs
  if (baseInputDim > 0) {
    ocs2::matrix_t R_base = ocs2::matrix_t::Zero(baseInputDim, baseInputDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R.base", R_base);
    R.topLeftCorner(baseInputDim, baseInputDim) = R_base;
  }

  matrix_t R_arm = matrix_t::Zero(armStateDim, armStateDim);
  loadData::loadEigenMatrix(taskFile, "inputCost.R.arm", R_arm);
  R.bottomRightCorner(armStateDim, armStateDim) = R_arm;

  std::cerr << "\n #### Input Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "inputCost.R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================\n";

  return std::make_unique<arm_interface::QuadraticInputCost>(std::move(R), armModuleInfo_.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 增加位置和速度软约束
std::unique_ptr<ocs2::StateInputCost> ARMInterface::getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                        const std::string& taskFile)
{
  // 信息读取
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  bool activateJointPositionLimit = true;
  ocs2::loadData::loadPtreeValue(pt, activateJointPositionLimit, "jointPositionLimits.activate", true);

  // 计算维度
  const int baseStateDim = armModuleInfo_.stateDim - armModuleInfo_.armDim;
  const int armStateDim = armModuleInfo_.armDim;
  const int baseInputDim = armModuleInfo_.inputDim - armModuleInfo_.armDim;
  const int armInputDim = armModuleInfo_.armDim;
  const auto& model = pinocchioInterface.getModel(); // 获取模型

  // Load position limits 加载位置限制
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
  if (activateJointPositionLimit) {
    ocs2::scalar_t muPositionLimits = 1e-2;
    ocs2::scalar_t deltaPositionLimits = 1e-3;

    // arm joint DOF limits from the parsed URDF  const auto& model = pinocchioInterface.getModel();
    const ocs2::vector_t lowerBound = model.lowerPositionLimit.tail(armStateDim);
    const ocs2::vector_t upperBound = model.upperPositionLimit.tail(armStateDim);


    // 从文件中加载惩罚参数
    std::cerr << "\n #### JointPositionLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### lowerBound: " << lowerBound.transpose() << '\n';
    std::cerr << " #### upperBound: " << upperBound.transpose() << '\n';
    ocs2::loadData::loadPtreeValue(pt, muPositionLimits, "jointPositionLimits.mu", true);
    ocs2::loadData::loadPtreeValue(pt, deltaPositionLimits, "jointPositionLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    // 设置惩罚参数
    stateLimits.reserve(armStateDim);
    for (int i = 0; i < armStateDim; ++i) {
      ocs2::StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = baseStateDim + i; // 跳过基础维度
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new ocs2::RelaxedBarrierPenalty({muPositionLimits, deltaPositionLimits}));
      stateLimits.push_back(std::move(boxConstraint));
    }
  }
  // load velocity limits 加载速度限制
  std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
  {
    // arm limits
    vector_t lowerBound = vector_t::Zero(armModuleInfo_.inputDim);
    vector_t upperBound = vector_t::Zero(armModuleInfo_.inputDim);
    scalar_t muVelocityLimits = 1e-2;
    scalar_t deltaVelocityLimits = 1e-3;

    // arm joint DOFs velocity limits
    vector_t lowerBoundArm = vector_t::Zero(armInputDim);
    vector_t upperBoundArm = vector_t::Zero(armInputDim);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound.arm", lowerBoundArm);
    loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound.arm", upperBoundArm);
    lowerBound.tail(armInputDim) = lowerBoundArm;
    upperBound.tail(armInputDim) = upperBoundArm;

    std::cerr << "\n #### JointVelocityLimits Settings: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'lowerBound':  " << lowerBound.transpose() << std::endl;
    std::cerr << " #### 'upperBound':  " << upperBound.transpose() << std::endl;
    loadData::loadPtreeValue(pt, muVelocityLimits, "jointVelocityLimits.mu", true);
    loadData::loadPtreeValue(pt, deltaVelocityLimits, "jointVelocityLimits.delta", true);
    std::cerr << " #### =============================================================================\n";

    inputLimits.reserve(armModuleInfo_.inputDim);
    std::cerr << " #### =============================FOR======================================\n";
    for (int i = 0; i < armModuleInfo_.inputDim; ++i) {
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;
      boxConstraint.index = i;
      boxConstraint.lowerBound = lowerBound(i);
      boxConstraint.upperBound = upperBound(i);
      boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muVelocityLimits, deltaVelocityLimits}));
      inputLimits.push_back(std::move(boxConstraint));
    }
  }

  auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
  boxConstraints->initializeOffset(0.0, vector_t::Zero(armModuleInfo_.stateDim), vector_t::Zero(armModuleInfo_.inputDim));
  return boxConstraints;
}

// 末端执行器约束，需要修改
std::unique_ptr<ocs2::StateCost> ARMInterface::getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                const std::string& taskFile, const std::string& prefix,
                                                                                bool usePreComputation, const std::string& libraryFolder,
                                                                                bool recompileLibraries,const std::string endEffectorName)
{
  ocs2::scalar_t muPosition = 1.0;
  ocs2::scalar_t muOrientation = 1.0;
  const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  ocs2::loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  ocs2::loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_== nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
  }
  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    std::cerr << " #### =============================================================================1\n";
    ARMPinocchioMapping pinocchioMapping(armModuleInfo_); // 系统输入到pinocchio的映射
    std::cerr << "Pinocchio model name: " << pinocchioInterface.getModel().name << "\n";
    std::cerr << "Number of joints: " << pinocchioInterface.getModel().nq << "\n";
    std::cerr << " #### =============================================================================2\n";
    PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, {endEffectorName});
    std::cerr << " #### =============================================================================3\n";
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_)); // 根据运动学 参考轨迹 执行器确定约束
  } else {
    ARMPinocchioMappingCppAd pinocchioMappingCppAd(armModuleInfo_);
    PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface, pinocchioMappingCppAd, {endEffectorName},
                                                     armModuleInfo_.stateDim, armModuleInfo_.inputDim,
                                                     prefix+"_end_effector_kinematics", libraryFolder, recompileLibraries, false);
    constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));
  }
  std::cerr << " #### =============================================================================\n";
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6); // 惩罚函数，前三个用于位置惩罚，后三个用于角度惩罚
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<QuadraticPenalty>(muPosition); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::make_unique<QuadraticPenalty>(muOrientation); });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
}

// 自碰撞约束
std::unique_ptr<ocs2::StateCost> ARMInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                                  const std::string& taskFile, const std::string& urdfFile,
                                                                                  const std::string& prefix, bool usePreComputation,
                                                                                  const std::string& libraryFolder,
                                                                                  bool recompileLibraries) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs; // 碰撞索引
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs; // 碰撞名称
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0; // 最小距离

  // 加载参数
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### SelfCollision Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", true);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", true);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, true);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, true);
  std::cerr << " #### =============================================================================\n";

  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);

  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs(); // 获取碰撞对数量
  std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";

  std::unique_ptr<StateConstraint> constraint;
  if (usePreComputation) {
    constraint = std::make_unique<ARMSelfCollisionConstraint>(ARMPinocchioMapping(armModuleInfo_),
                                                                            std::move(geometryInterface), minimumDistance);
  } else {
    constraint = std::make_unique<SelfCollisionConstraintCppAd>(
        pinocchioInterface, ARMPinocchioMapping(armModuleInfo_), std::move(geometryInterface), minimumDistance,
        "self_collision", libraryFolder, recompileLibraries, false);
  }

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}


}