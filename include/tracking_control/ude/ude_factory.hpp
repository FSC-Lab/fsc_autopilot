#ifndef TRACKING_CONTROL_UDE_UDE_FACTORY_HPP_
#define TRACKING_CONTROL_UDE_UDE_FACTORY_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "tracking_control/logging.hpp"
#include "tracking_control/ude/ude_base.hpp"

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
