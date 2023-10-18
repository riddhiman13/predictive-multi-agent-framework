#include <bimanual_planning_ros/controller_type.h>

ControllerType::ControllerType(){};
ControllerType::ControllerType(ControllerType::Type type) : t_(type){};
ControllerType::ControllerType(const std::string& type) {
  if (!type.compare("relative_pose")) {
    t_ = ControllerType::RELATIVE_POSE;
  } else if (!type.compare("whole_body_avoidance")) {
    t_ = ControllerType::WHOLE_BODY_AVOIDANCE;
  } else if (!type.compare("absolute_position")) {
    t_ = ControllerType::ABSOLUTE_POSITION;
  } else if (!type.compare("ee_tilt")) {
    t_ = ControllerType::EE_TILT;
  } else if (!type.compare("rotational_admittance")) {
    t_ = ControllerType::ROTATIONAL_ADMITTANCE;
  } else if (!type.compare("joint_limit_avoidance")) {
    t_ = ControllerType::JOINT_LIMIT_AVOIDANCE;
  } else {
    t_ = ControllerType::INVALID;
  }
}
ControllerType::operator Type() { return t_; };
