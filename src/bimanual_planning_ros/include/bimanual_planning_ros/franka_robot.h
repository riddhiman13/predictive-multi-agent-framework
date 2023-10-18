#pragma once

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics {

class Panda {
 public:
  static DQ_SerialManipulator kinematics(double r_B_O[3], double B_Q_O[4]);
};

}  // namespace DQ_robotics
