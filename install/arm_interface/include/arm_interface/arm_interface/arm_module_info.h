#pragma once

// C/C++
#include <string>
#include <vector>
namespace arm_interface{
struct ARMModuleInfo{
  size_t stateDim;
  size_t inputDim;
  size_t armDim;

  std::string baseFrame;
  std::string eeFrame;
  // std::string ereFrame;
  std::vector<std::string> dofNames;
};
}