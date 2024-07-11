#ifndef TRACKING_CONTROL_CORE_PARAMETER_BASE_HPP_
#define TRACKING_CONTROL_CORE_PARAMETER_BASE_HPP_

#include <string>

#include "tracking_control/core/logger_base.hpp"

namespace fsc {
class ParameterLoaderBase {
 public:
  virtual ~ParameterLoaderBase() = default;

  template <typename T>
  T param(const std::string& key, const T& fallback) const {
    if (T value; getParam(key, value)) {
      return value;
    }
    return fallback;
  }

  [[nodiscard]] virtual bool hasParam(const std::string& key) const {
    return false;
  }

  virtual bool getParam(const std::string& key, bool& value) const = 0;

  virtual bool getParam(const std::string& key, int& value) const = 0;

  virtual bool getParam(const std::string& key, double& value) const = 0;

  virtual bool getParam(const std::string& key, std::string& value) const = 0;
};

class ParameterBase {
 public:
  virtual ~ParameterBase() = default;

  [[nodiscard]] virtual bool valid() const = 0;

  [[nodiscard]] virtual std::string parameterFor() const = 0;

  [[nodiscard]] std::string name() const {
    return parameterFor() + ".parameters";
  }

  bool load(const ParameterLoaderBase& loader) { return load(loader, nullptr); }

  virtual bool load(const ParameterLoaderBase& loader, LoggerBase* logger) = 0;

  [[nodiscard]] virtual std::string toString() const = 0;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_CORE_PARAMETER_BASE_HPP_
