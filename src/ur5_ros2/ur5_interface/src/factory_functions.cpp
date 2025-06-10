#include "ur5_interface/factory_functions.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include "ur5_interface/ur5_module_info.h"

namespace ur5_interface {
using namespace ocs2;

ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const std::vector<std::string>& jointNames)
{
  using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;
  const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
  if (!urdfTree) {
    throw std::runtime_error("URDF解析失败");
  }
  // remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for(auto jn: jointNames){
    std::cerr << "joint name: " << jn << std::endl;
  }
  for (joint_pair_t& jointPair : newModel->joints_) {
    std::cerr << "model joint  name: " << jointPair.first<< std::endl;
    if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
      std::cerr << "Found joint name: " << jointPair.first << std::endl;
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }
  return getPinocchioInterfaceFromUrdfModel(newModel);
}

// 获取构建运动学的器人的维度
UR5ModuleInfo createUR5ModuleInfo(const ocs2::PinocchioInterface& interface, const std::string& baseFrame, const std::string& eeFrame)
{
  const auto& model = interface.getModel();
  UR5ModuleInfo info;
  info.stateDim = model.nq;
  std::cerr << "\n #### nq:" << info.stateDim << std::endl;
  info.inputDim = info.stateDim;
  std::cerr << "\n #### inputDim:" << info.inputDim << std::endl;
  info.armDim = info.inputDim;
  std::cerr << "\n #### armDim:" << info.armDim << std::endl;
  info.baseFrame = baseFrame;
  std::cerr << "\n #### baseFrame:" << info.baseFrame << std::endl;
  info.eeFrame = eeFrame;
  std::cerr << "\n #### eeFrame:" << info.eeFrame << std::endl;

  const auto& jointNames = model.names;
  info.dofNames = std::vector<std::string>(jointNames.begin(),jointNames.end()); // 左右臂关节名称
  return info;
}

} //namespace arm_interfac