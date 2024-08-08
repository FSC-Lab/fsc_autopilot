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

#ifndef FSC_AUTOPILOT_LQG_LQG_FACTORY_HPP_
#define FSC_AUTOPILOT_LQG_LQG_FACTORY_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/lqg/lqg_base.hpp"

namespace fsc {
class LQGFactory {
 public:
  using LQGUniquePtr = std::unique_ptr<LQGBase>;
  using LQGCreator = std::function<LQGUniquePtr()>;

  LQGFactory() = delete;

  static bool Register(std::string name, LQGCreator creator);

  static LQGUniquePtr Create(const std::string& name,
                             LoggerBase* logger = nullptr);

  template <typename OutputIter>
  static void GetRegistryKeys(OutputIter first) {
    const auto sent = registry_.cend();
    for (auto it = registry_.cbegin(); it != sent; ++it, ++first) {
      *first = it->first;
    }
  }

 private:
  inline static std::unordered_map<std::string, LQGCreator> registry_;
};

template <typename T>
class LQGRegistrar {
 public:
  static_assert(
      std::is_base_of_v<LQGBase, T>,
      "This registrar can only register classes derived from LQGBase");

  using LQGUniquePtr = LQGFactory::LQGUniquePtr;

  explicit LQGRegistrar(std::string name) {
    LQGFactory::Register(std::move(name), createImpl);
  }

 private:
  static LQGUniquePtr createImpl() { return std::make_unique<T>(); }
};

#define REGISTER_LQG(LQG, name) \
  static const LQGRegistrar<LQG> kLqgRegistrarFor##LQG((name))

}  // namespace fsc

#endif  // FSC_AUTOPILOT_LQG_LQG_FACTORY_HPP_
