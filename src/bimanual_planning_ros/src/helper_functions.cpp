#include "bimanual_planning_ros/helper_functions.h"

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
// #include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Constants.h>

Eigen::Vector3d makeRandomVector() {
  Eigen::Vector3d ret;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-1.0, 1.0);
  ret << dis(gen), dis(gen), dis(gen);
  return ret;
}