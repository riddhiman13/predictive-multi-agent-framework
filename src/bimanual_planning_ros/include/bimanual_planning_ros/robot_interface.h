#pragma once
#include <vector>

class RobotInterface {
 public:
  virtual void init() {};
  virtual void openGripper(const bool& left, const double& width) {};
  virtual void openGrippers(const double& width) {};
  virtual void closeGripper(const bool& left) {};
  virtual void closeGrippers() {};
  virtual void setReflexThresholds(const std::vector<double>& F_max,
                                   const std::vector<double>& tau_max) {};
};
