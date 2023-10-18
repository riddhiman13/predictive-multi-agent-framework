#pragma once

#include <memory>
#include <functional>

#include <ros/ros.h>

class ParameterInterface {
 public:
  virtual bool check(const XmlRpc::XmlRpcValue& params,
                     const std::string& parent) = 0;
  virtual bool isPresent(const XmlRpc::XmlRpcValue& params) const = 0;
  virtual void load(const XmlRpc::XmlRpcValue& params) = 0;
  virtual const std::string& getName() const = 0;
  virtual std::unique_ptr<ParameterInterface> clone() const = 0;
  virtual std::unique_ptr<ParameterInterface> clone(
      bool optional_override) const = 0;
  virtual ~ParameterInterface() {};
};

template<typename T>
class ParameterBase : public ParameterInterface {
 public:
  ParameterBase(const std::string& name, T& storage, bool optional=false,
      std::function<bool(const T&, const std::string&)> value_check =
      [](const T&, const std::string&)->bool {return true;}) :
      storage_(storage), name_(name), optional_(optional),
      valueCheck(value_check) {};
  bool check(const XmlRpc::XmlRpcValue& params,
             const std::string& parent) override {
    if (!params.hasMember(name_)) {
      if (!optional_) {
        ROS_ERROR("Parameter %s could not be found in %s.", name_.c_str(),
                  parent.c_str());
      }
      return optional_;
    }
    if (checkType(params, parent)) {
      T tmp = storage_;
      load(params, tmp);
      return valueCheck(tmp, parent);
    } else {
      return false;
    }
  }
  void load(const XmlRpc::XmlRpcValue& params) override {
    load(params, storage_);
  }
  bool isPresent(const XmlRpc::XmlRpcValue& params) const override {
    return params.hasMember(name_);
  }
  const std::string& getName() const override {
    return name_;
  }
 protected:
  virtual void load(const XmlRpc::XmlRpcValue& params, T& storage) = 0;
  virtual bool checkType(const XmlRpc::XmlRpcValue& params,
                         const std::string& parent) = 0;
  double getNumericRPCValue(XmlRpc::XmlRpcValue v) {
    if (v.getType() == XmlRpc::XmlRpcValue::Type::TypeInt) {
      return (int &)v;
    } else if (v.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble) {
      return (double &)v;
    } else if (v.getType() == XmlRpc::XmlRpcValue::Type::TypeBoolean) {
      return (bool &)v;
    } else {
      ROS_ERROR("Wrong type detected for numeric value in yaml.");
      ROS_ERROR_STREAM("Value is: " << v.toXml());
      throw "Type error";
    }
  }
  T& storage_;
  std::string name_;
  bool optional_;
  std::function<bool(const T& value, const std::string& parent)> valueCheck;
};

template <typename T>
class StringParameter : public ParameterBase<T> {
 public:
  StringParameter(const std::string& name, T& storage, bool optional=false,
      std::function<bool(const T&, const std::string&)> value_check =
      [](const T&, const std::string&)->bool {return true;}) :
      ParameterBase<T>(name, storage, optional, value_check) {};
  std::unique_ptr<ParameterInterface> clone() const override {
    return std::make_unique<StringParameter>(this->name_, this->storage_,
                                             this->optional_, this->valueCheck);
  }
  std::unique_ptr<ParameterInterface> clone(
      bool optional_override) const override {
    return std::make_unique<StringParameter>(this->name_, this->storage_,
        optional_override, this->valueCheck);
  }
 protected:
  void load(const XmlRpc::XmlRpcValue& params, T& storage) {
    storage = (std::string&) params[this->name_];
  }
  bool checkType(const XmlRpc::XmlRpcValue& params,
                 const std::string& parent) {
    if (params[this->name_].getType() !=
        XmlRpc::XmlRpcValue::Type::TypeString) {
      ROS_ERROR("Parameter %s of %s has wrong type, must be string.",
                this->name_.c_str(), parent.c_str());
      return false;
    } else {
      return true;
    }
  }
};

template <typename T>
class ScalarParameter : public ParameterBase<T> {
 public:
  ScalarParameter(const std::string& name, T& storage, bool optional=false,
      std::function<bool(const T&, const std::string&)> value_check =
      [](const T&, const std::string&)->bool {return true;}) :
      ParameterBase<T>(name, storage, optional, value_check) {};
  std::unique_ptr<ParameterInterface> clone() const override {
    return std::make_unique<ScalarParameter>(this->name_, this->storage_,
                                             this->optional_, this->valueCheck);
  }
  std::unique_ptr<ParameterInterface> clone(
      bool optional_override) const override {
    return std::make_unique<ScalarParameter>(this->name_, this->storage_,
        optional_override, this->valueCheck);
  }
 protected:
  void load(const XmlRpc::XmlRpcValue& params, T& storage) {
    storage = this->getNumericRPCValue(params[this->name_]);
  }
  bool checkType(const XmlRpc::XmlRpcValue& params,
                 const std::string& parent) {
    if (params[this->name_].getType() !=
        XmlRpc::XmlRpcValue::Type::TypeDouble &&
        params[this->name_].getType() != XmlRpc::XmlRpcValue::Type::TypeInt &&
        params[this->name_].getType() !=
        XmlRpc::XmlRpcValue::Type::TypeBoolean) {
      ROS_ERROR("Parameter %s of %s has wrong type, must be numeric and "
                "scalar.", this->name_.c_str(), parent.c_str());
      return false;
    } else {
      return true;
    }
  }
};

template <typename T>
class ArrayParameter : public ParameterBase<T> {
 public:
  ArrayParameter(const std::string& name, T& storage, bool optional=false,
      std::function<bool(const T&, const std::string&)> value_check =
      [](const T&, const std::string&)->bool {return true;}) :
      ParameterBase<T>(name, storage, optional, value_check) {};
  std::unique_ptr<ParameterInterface> clone() const override {
    return std::make_unique<ArrayParameter>(this->name_, this->storage_,
                                             this->optional_, this->valueCheck);
  }
  std::unique_ptr<ParameterInterface> clone(
      bool optional_override) const override {
    return std::make_unique<ArrayParameter>(this->name_, this->storage_,
        optional_override, this->valueCheck);
  }
 protected:
  void load(const XmlRpc::XmlRpcValue& params, T& storage) {
    for (int i=0;i<storage.size();++i) {
      storage[i] = this->getNumericRPCValue(params[this->name_][i]);
    }
  }
  bool checkType(const XmlRpc::XmlRpcValue& params,
                 const std::string& parent) {
    bool success = true;
    if (this->storage_.size() != params[this->name_].size()) {
      ROS_ERROR("Storage of parameter %s of %s has size %i, but specified "
                "parameter has size %i.", this->name_.c_str(), parent.c_str(),
                this->storage_.size(), params[this->name_].size());
      success = false;
    }
    for (int i=0;i<params[this->name_].size();++i) {
      if (params[this->name_][i].getType() !=
          XmlRpc::XmlRpcValue::Type::TypeDouble &&
          params[this->name_][i].getType() !=
          XmlRpc::XmlRpcValue::Type::TypeInt &&
          params[this->name_][i].getType() !=
          XmlRpc::XmlRpcValue::Type::TypeBoolean) {
        ROS_ERROR("Field %i of parameter %s of %s has wrong type, must "
                  "be numeric and scalar.", i, this->name_.c_str(),
                  parent.c_str());
        success = false;
      }
    }
    return success;
  }
};

class ParameterManager : public ParameterInterface {
 public:
  ParameterManager(const std::string& name, bool optional=false);
  ParameterManager(const ParameterManager& pm);
  ParameterManager& operator=(const ParameterManager& pm);
  ParameterManager(const ParameterManager& pm, bool optional);
  void addHook(const std::function<void()>& hook, const std::string& name);
  void insertHookFront(const std::function<void()>& hook,
                       const std::string& name);
  void addParameter(const ParameterInterface& p,
                    std::vector<std::string>&& post_hooks={},
                    std::vector<std::string>&& pre_hooks={});
  void insertParameterFront(const ParameterInterface& p,
                    std::vector<std::string>&& post_hooks={},
                    std::vector<std::string>&& pre_hooks={});
  bool check(const XmlRpc::XmlRpcValue& params,
             const std::string& parent) override;
  bool checkParameters(const XmlRpc::XmlRpcValue& params) const;
  bool checkParameters(const XmlRpc::XmlRpcValue& params,
                       const std::string& parent) const;
  void load(const XmlRpc::XmlRpcValue& params) override;
  void loadParameters(const XmlRpc::XmlRpcValue& params);
  const std::string& getName() const override;
  bool isPresent(const XmlRpc::XmlRpcValue& params) const override;
  std::unique_ptr<ParameterInterface> clone() const override;
  std::unique_ptr<ParameterInterface> clone(
      bool optional_override) const override;
  void printHooks();
 private:
  void getParameterValues(const XmlRpc::XmlRpcValue& params);
  std::vector<std::unique_ptr<ParameterInterface>> parameters_;
  std::vector<std::vector<std::string>> pre_hooked_to_;
  std::vector<std::vector<std::string>> post_hooked_to_;
  std::map<std::string, std::function<void()>> hooks_;
  std::vector<std::string> hooks_ordered_;
  std::string name_;
  bool optional_;
};


