#pragma once

#include <string>

class GoalType {
 public:
  enum Type {KEY, GESTURE, GOTO, PLAN, GRASP, RELEASE, ERROR_RECOVERY,
             START_LOGGING, STOP_LOGGING, INVALID};
  GoalType();
  GoalType(GoalType::Type type);
  GoalType(const std::string& type);
  operator Type();
 private:
  Type t_;
};
