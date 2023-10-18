#include <bimanual_planning_ros/goal_type.h>

GoalType::GoalType() {};
GoalType::GoalType(GoalType::Type type) : t_(type) {};
GoalType::GoalType(const std::string& type) {
  if (!type.compare("key")) {
    t_ = GoalType::KEY;
  } else if (!type.compare("gesture")) {
    t_ = GoalType::GESTURE;
  } else if (!type.compare("goto")) {
    t_ = GoalType::GOTO;
  } else if (!type.compare("plan")) {
    t_ = GoalType::PLAN;
  } else if (!type.compare("grasp")) {
    t_ = GoalType::GRASP;
  } else if (!type.compare("release")) {
    t_ = GoalType::RELEASE;
  } else if (!type.compare("error_recovery")) {
    t_ = GoalType::ERROR_RECOVERY;
  } else if (!type.compare("start_logging")) {
    t_ = GoalType::START_LOGGING;
  } else if (!type.compare("stop_logging")) {
    t_ = GoalType::STOP_LOGGING;
  } else {
    t_ = GoalType::INVALID;
  }
}
GoalType::operator Type() {return t_;};
