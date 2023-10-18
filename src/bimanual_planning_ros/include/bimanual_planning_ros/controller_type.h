#pragma once

#include <string>

class ControllerType {
 public:
  enum Type {
    RELATIVE_POSE,
    WHOLE_BODY_AVOIDANCE,
    ABSOLUTE_POSITION,
    EE_TILT,
    ROTATIONAL_ADMITTANCE,
    JOINT_LIMIT_AVOIDANCE,
    INVALID
  };
  ControllerType();
  ControllerType(ControllerType::Type type);
  ControllerType(const std::string& type);
  operator Type();

 private:
  Type t_;
};
