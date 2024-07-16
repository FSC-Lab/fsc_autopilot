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

#ifndef TRACKING_CONTROL_UDE_UDE_FACTORY_HPP_
#define TRACKING_CONTROL_UDE_UDE_FACTORY_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/ude/ude_base.hpp"

namespace fsc {
class UDEFactory {
 public:
  using UDEUniquePtr = std::unique_ptr<UDEBase>;
  using ParameterBaseSharedPtr = std::shared_ptr<UDEParameters>;
  using UDECreator = std::function<UDEUniquePtr(ParameterBaseSharedPtr)>;

  UDEFactory() = delete;

  static bool Register(std::string name, UDECreator creator);

  static UDEUniquePtr Create(ParameterBaseSharedPtr params,
                             LoggerBase* logger = nullptr);

  template <typename OutputIter>
  static void GetRegistryKeys(OutputIter first) {
    const auto sent = registry_.cend();
    for (auto it = registry_.cbegin(); it != sent; ++it, ++first) {
      *first = it->first;
    }
  }

 private:
  inline static std::unordered_map<std::string, UDECreator> registry_;
};

template <typename T>
class UDERegistrar {
 public:
  static_assert(
      std::is_base_of_v<UDEBase, T>,
      "This registrar can only register classes derived from UDEBase");

  using ParameterBaseSharedPtr = UDEFactory::ParameterBaseSharedPtr;
  using UDEUniquePtr = UDEFactory::UDEUniquePtr;

  explicit UDERegistrar(std::string name) {
    UDEFactory::Register(std::move(name), createImpl);
  }

 private:
  static UDEUniquePtr createImpl(ParameterBaseSharedPtr params) {
    return std::make_unique<T>(std::move(params));
  }
};

#define REGISTER_UDE(UDE, name) \
  static const UDERegistrar<UDE> kUdeRegistrarFor##UDE((name))

}  // namespace fsc

#endif  // TRACKING_CONTROL_UDE_UDE_FACTORY_HPP_