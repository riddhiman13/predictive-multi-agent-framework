#include <bimanual_planning_ros/parameter_manager.h>

ParameterManager::ParameterManager(const std::string& name, bool optional) {
  name_ = name;
  optional_ = optional;
}

ParameterManager::ParameterManager(const ParameterManager& pm) {
  for (int i=0;i<pm.parameters_.size();++i) {
    parameters_.push_back(pm.parameters_[i]->clone());
  }
  hooks_ = pm.hooks_;
  post_hooked_to_ = pm.post_hooked_to_;
  pre_hooked_to_ = pm.pre_hooked_to_;
  hooks_ordered_ = pm.hooks_ordered_;
  name_ = pm.name_;
  optional_ = pm.optional_;
}

ParameterManager::ParameterManager(const ParameterManager& pm, bool optional) {
  for (int i=0;i<pm.parameters_.size();++i) {
    parameters_.push_back(pm.parameters_[i]->clone(optional));
  }
  hooks_ = pm.hooks_;
  post_hooked_to_ = pm.post_hooked_to_;
  pre_hooked_to_ = pm.pre_hooked_to_;
  hooks_ordered_ = pm.hooks_ordered_;
  name_ = pm.name_;
  optional_ = optional;
}

ParameterManager& ParameterManager::operator=(const ParameterManager& pm) {
  for (int i=0;i<pm.parameters_.size();++i) {
    parameters_.push_back(pm.parameters_[i]->clone());
  }
  hooks_ = pm.hooks_;
  post_hooked_to_ = pm.post_hooked_to_;
  pre_hooked_to_ = pm.pre_hooked_to_;
  hooks_ordered_ = pm.hooks_ordered_;
  return *this;
}

std::unique_ptr<ParameterInterface> ParameterManager::clone() const {
  return std::make_unique<ParameterManager>(ParameterManager(*this));
}

std::unique_ptr<ParameterInterface> ParameterManager::clone(
    bool optional_override) const {
  return std::make_unique<ParameterManager>(ParameterManager(*this,
      optional_override));
}

void ParameterManager::addHook(const std::function<void()>& hook,
                               const std::string& name) {
  if (hooks_.find(name) == hooks_.end()) {
    hooks_[name] = hook;
    hooks_ordered_.push_back(name);
  } else {
    ROS_ERROR("Hook %s already exists, aborting.", name.c_str());
    throw "Hook double defined";
  }
}

void ParameterManager::insertHookFront(const std::function<void()>& hook,
                                       const std::string& name) {
  if (hooks_.find(name) == hooks_.end()) {
    hooks_[name] = hook;
    hooks_ordered_.insert(hooks_ordered_.begin(), name);
  } else {
    ROS_ERROR("Hook %s already exists, aborting.", name.c_str());
    throw "Hook double defined";
  }
}

void ParameterManager::addParameter(const ParameterInterface& p,
                                    std::vector<std::string>&& post_hooks,
                                    std::vector<std::string>&& pre_hooks) {
  parameters_.push_back(p.clone());
  pre_hooked_to_.push_back(pre_hooks);
  post_hooked_to_.push_back(post_hooks);
}

void ParameterManager::insertParameterFront(const ParameterInterface& p,
    std::vector<std::string>&& post_hooks,
    std::vector<std::string>&& pre_hooks) {
  parameters_.insert(parameters_.begin(), p.clone());
  pre_hooked_to_.insert(pre_hooked_to_.begin(), pre_hooks);
  post_hooked_to_.insert(post_hooked_to_.begin(), post_hooks);
}

const std::string& ParameterManager::getName() const {
  return name_;
}

bool ParameterManager::isPresent(const XmlRpc::XmlRpcValue& params) const {
  return params.hasMember(name_);
}

bool ParameterManager::checkParameters(
    const XmlRpc::XmlRpcValue& params) const {
  return checkParameters(params, name_);
}

bool ParameterManager::checkParameters(
    const XmlRpc::XmlRpcValue& params, const std::string& parent) const {
  for (auto i = params.begin(); i != params.end(); ++i) {
    bool used = false;
    for (int j=0;j<parameters_.size();++j) {
      if (i->first == parameters_[j]->getName()) {
        used = true;
        break;
      }
    }
    if (!used) {
      ROS_WARN("Unused parameter %s found in %s.", i->first.c_str(),
               parent.c_str());
    }
  }
  bool success = true;
  for (int i=0;i<parameters_.size();++i) {
    if (!parameters_[i]->check(params, parent)) {
      success = false;
    }
  }
  return success;
}

bool ParameterManager::check(const XmlRpc::XmlRpcValue& params,
                             const std::string& parent) {
  if (isPresent(params)) {
    if (params[name_].getType() == XmlRpc::XmlRpcValue::Type::TypeArray) {
      bool success = true;
      for (int i=0;i<params[name_].size();++i) {
        success &= checkParameters(params[name_][i], name_ + "[" +
                                   std::to_string(i) + "] in " + parent);
      }
      return success;
    } else {
      return checkParameters(params[name_], name_ + " in " + parent);
    }
  } else {
    if (!optional_) {
      ROS_ERROR("Parameter %s could not be found in %s.", name_.c_str(),
                parent.c_str());
    }
    return optional_;
  }
}

void ParameterManager::loadParameters(const XmlRpc::XmlRpcValue& params) {
  std::set<std::string> hook_active;
  for (int i=0;i<parameters_.size();++i) {
    if (parameters_[i]->isPresent(params)) {
      for (int j=0;j<pre_hooked_to_[i].size();++j) {
        hooks_.at(pre_hooked_to_[i][j])();
      }
      for (int j=0;j<post_hooked_to_[i].size();++j) {
        hook_active.insert(post_hooked_to_[i][j]);
      }
      parameters_[i]->load(params);
    }
  }
  for (int i=0;i<hooks_ordered_.size();++i) {
    if (hook_active.find(hooks_ordered_[i]) != hook_active.end()) {
      hooks_.at(hooks_ordered_[i])();
    }
  }
}

void ParameterManager::load(const XmlRpc::XmlRpcValue& params) {
  if (isPresent(params)) {
    if (params[name_].getType() == XmlRpc::XmlRpcValue::Type::TypeArray) {
      for (int i=0;i<params[name_].size();++i) {
        loadParameters(params[name_][i]);
      }
    } else {
      loadParameters(params[name_]);
    }
  }
}

void ParameterManager::printHooks() {
  for (int i=0;i<parameters_.size();++i) {
    std::stringstream s;
    if (post_hooked_to_[i].size()) {
      s << "Parameter " << parameters_[i]->getName() << " is post hooked to ";
      int j;
      for (j=0;j<post_hooked_to_[i].size()-1;++j) {
        s << post_hooked_to_[i][j] << " , ";
      }
      s << post_hooked_to_[i][j];
      ROS_INFO("%s", s.str().c_str());
    } else {
      ROS_INFO("Parameter %s has no post hooks.",
               parameters_[i]->getName().c_str());
    }
    s.str("");
    if (pre_hooked_to_[i].size()) {
      s << "Parameter " << parameters_[i]->getName() << " is pre hooked to ";
      int j;
      for (j=0;j<pre_hooked_to_[i].size()-1;++j) {
        s << pre_hooked_to_[i][j] << " , ";
      }
      s << pre_hooked_to_[i][j];
      ROS_INFO("%s", s.str().c_str());
    } else {
      ROS_INFO("Parameter %s has no pre hooks.",
               parameters_[i]->getName().c_str());
    }
  }
  ROS_INFO("Post hooks for %s are defined and executed in this order:",
           name_.c_str());
  for (int i=0;i<hooks_ordered_.size();++i) {
    ROS_INFO("%i: %s.", i, hooks_ordered_[i].c_str());
  }
}
