// Copyright © 2024 FSC Lab
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef FSC_AUTOPILOT_CORE_PARAMETER_BASE_HPP_
#define FSC_AUTOPILOT_CORE_PARAMETER_BASE_HPP_

#include <memory>
#include <string>

#include "fsc_autopilot/core/logger_base.hpp"

namespace fsc {
class ParameterLoaderBase {
 public:
  virtual ~ParameterLoaderBase() = default;

  [[nodiscard]] virtual std::shared_ptr<ParameterLoaderBase> getChildLoader(
      const std::string& ns) const {
    return nullptr;
  }

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

#endif  // FSC_AUTOPILOT_CORE_PARAMETER_BASE_HPP_
