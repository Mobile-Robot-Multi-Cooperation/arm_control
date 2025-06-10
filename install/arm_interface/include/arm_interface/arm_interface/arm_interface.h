#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "arm_interface/factory_functions.h"
#include "arm_interface/arm_module_info.h"
#include "arm_interface/arm_precomputation.h"
#include "arm_interface/arm_pinocchio_mapping.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace arm_interface{
using namespace ocs2; // 引入ocs2空间，可使用内部类和函数
class ARMInterface final: public RobotInterface{
public:
  ARMInterface(const std::string& taskFile, const std::string& libraryFolder,
                                      const std::string& urdfFile);
  ~ARMInterface() override = default;
  const ocs2::vector_t & getInitializerState(){ return initialState_; }

  ocs2::ddp::Settings& ddpSettings(){ return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings(){ return mpcSettings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override {return problem_;}

  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override {return referenceManagerPtr_;}

  const ocs2::Initializer& getInitializer() const override {return *initializerPtr_;}

  const ocs2::RolloutBase& getRollout() const {return *rolloutPtr_;}

  const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_;}

  const ARMModuleInfo& getARMModuelInfo() const {return armModuleInfo_;}
private:
  std::unique_ptr<ocs2::StateInputCost> getQuadraticInputCost(const std::string& taskFile);
  std::unique_ptr<ocs2::StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries,const std::string endEffectorName);
  std::unique_ptr<ocs2::StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& urdfFile, const std::string& prefix, bool useCaching,
                                                        const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

  std::unique_ptr<ocs2::StateCost> getBaseConstraint(const PinocchioInterface& pinocchioInterface,const std::string& taskFile,
                                                      const std::string& prefix,bool usePreComputation, const std::string& libraryFolder,
                                                      bool recompileLibraries);

  ocs2::ddp::Settings ddpSettings_;
  ocs2::mpc::Settings mpcSettings_;

  ocs2::OptimalControlProblem problem_;
  std::shared_ptr<ocs2::ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
  std::unique_ptr<ocs2::Initializer> initializerPtr_;

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
  std::unique_ptr<ocs2::PinocchioInterface> pinocchioNoBaseInterfacePtr_;
  ARMModuleInfo armModuleInfo_;


  ocs2::vector_t initialState_;

};

}// namespacce