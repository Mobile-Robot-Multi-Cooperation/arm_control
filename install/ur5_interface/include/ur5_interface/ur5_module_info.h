#pragma once

// C/C++
#include <string>
#include <vector>
namespace ur5_interface{
struct UR5ModuleInfo{
  size_t stateDim;
  size_t inputDim;
  size_t armDim;

  std::string baseFrame;
  std::string eeFrame;
  // std::string ereFrame;
  std::vector<std::string> dofNames;
};
}