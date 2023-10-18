#ifndef HELPER_FUNCTION_H
#define HELPER_FUNCTION_H

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

#include <random>

#include "eigen3/Eigen/Dense"

using DQ_robotics::DQ_CooperativeDualTaskSpace;
using DQ_robotics::DQ_SerialManipulator;

Eigen::Vector3d makeRandomVector();

#endif
