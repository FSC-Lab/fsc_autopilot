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

#ifndef FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_FACTORY_HPP_
#define FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_FACTORY_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "fsc_autopilot/attitude_control/attitude_controller_base.hpp"
#include "fsc_autopilot/core/factory_support.hpp"

namespace fsc {
class AttitudeControllerFactory {
 public:
  using ProductUniquePtr = std::unique_ptr<AttitudeControllerBase>;
  using Creator = std::function<ProductUniquePtr()>;

  AttitudeControllerFactory() = delete;

  static bool Register(std::string name, Creator creator);

  static ProductUniquePtr Create(const std::string& name, LoggerBase& logger);

  template <typename OutputIter>
  static void GetRegistryKeys(OutputIter first) {
    const auto sent = registry_.cend();
    for (auto it = registry_.cbegin(); it != sent; ++it, ++first) {
      *first = it->first;
    }
  }

 private:
  inline static std::unordered_map<std::string, Creator> registry_;
};

template <typename T>
using AttitudeControllerRegistrar = Registrar<T, AttitudeControllerFactory>;

#define REGISTER_ATTITUDE_CONTROLLER(AC, name) \
  static const AttitudeControllerRegistrar<AC> kUdeRegistrarFor##AC((name))

}  // namespace fsc

#endif  // FSC_AUTOPILOT_ATTITUDE_CONTROL_ATTITUDE_CONTROLLER_FACTORY_HPP_
