#include "tracking_control/ude/ude_factory.hpp"

namespace fsc {
bool UDEFactory::Register(std::string name, UDECreator creator) {
  auto [_, success] =
      registry_.try_emplace(std::move(name), std::move(creator));
  return success;
}

UDEFactory::UDEUniquePtr UDEFactory::Create(ParameterBaseSharedPtr params) {
  if (auto it = registry_.find(params->parameterFor()); it != registry_.end()) {
    return it->second(std::move(params));
  }
  return {};
}
}  // namespace fsc
