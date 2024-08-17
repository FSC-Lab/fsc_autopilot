#ifndef FSC_AUTOPILOT_CORE_FACTORY_SUPPORT_HPP_
#define FSC_AUTOPILOT_CORE_FACTORY_SUPPORT_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include "fsc_autopilot/utils/meta.hpp"

namespace fsc {
template <typename T, typename Factory>
class Registrar {
 public:
  using ProductUniquePtr = typename Factory::ProductUniquePtr;
  using ProductBase = typename ProductUniquePtr::element_type;

  static_assert(
      std::is_base_of_v<ProductBase, T>,
      "This registrar can only register classes derived from UDEBase");

  explicit Registrar(std::string name) {
    Factory::Register(FWD(name), std::make_unique<T>);
  }
};

}  // namespace fsc

#endif  // FSC_AUTOPILOT_CORE_FACTORY_SUPPORT_HPP_
