#include <bimanual_planning_ros/end_condition.h>

EndCondition::EndCondition() {};
EndCondition::EndCondition(EndCondition::Type type) : t_(type) {};
EndCondition::EndCondition(const std::string& type) {
  if (!type.compare("none")) {
    t_ = EndCondition::NONE;
  } else if (!type.compare("reached")) {
    t_ = EndCondition::REACHED;
  } else if (!type.compare("contact")) {
    t_ = EndCondition::CONTACT;
  } else if (!type.compare("endless")) {
    t_ = EndCondition::ENDLESS;
  } else {
    t_ = EndCondition::INVALID;
  }
}
EndCondition::operator Type() {return t_;};
