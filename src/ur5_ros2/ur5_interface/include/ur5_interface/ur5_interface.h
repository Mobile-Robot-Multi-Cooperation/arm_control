#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "ur5_interface/factory_functions.h"
#include "ur5_interface/ur5_module_info.h"
#include "ur5_interface/ur5_precomputation.h"
#include "ur5_interface/ur5_pinocchio_mapping.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ur5_interface{
using namespace ocs2; // 引入ocs2空间，可使用内部类和函数
class UR5Interface final: public RobotInterface{
public:
  UR5Interface(const std::string& taskFile, const std::string& libraryFolder,
                                      const std::string& urdfFile);
  ~UR5Interface() override = default;
  const ocs2::vector_t & getInitializerState(){ return initialState_; }

  ocs2::ddp::Settings& ddpSettings(){ return ddpSettings_; }

  ocs2::mpc::Settings& mpcSettings(){ return mpcSettings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override {return problem_;}

  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override {return referenceManagerPtr_;}

  const ocs2::Initializer& getInitializer() const override {return *initializerPtr_;}

  const ocs2::RolloutBase& getRollout() const {return *rolloutPtr_;}

  const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_;}

  const UR5ModuleInfo& getUR5ModuelInfo() const {return ur5ModuleInfo_;}
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
  UR5ModuleInfo ur5ModuleInfo_;


  ocs2::vector_t initialState_;

};

}// namespacce