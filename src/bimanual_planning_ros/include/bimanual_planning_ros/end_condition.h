#pragma once

#include <string>

class EndCondition {
 public:
  enum Type {NONE, CONTACT, REACHED, ENDLESS, INVALID};
  EndCondition();
  EndCondition(EndCondition::Type type);
  EndCondition(const std::string& type);
  operator Type();
 private:
  Type t_;
};
