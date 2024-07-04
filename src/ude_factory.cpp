#include "tracking_control/ude/ude_factory.hpp"

#include "tracking_control/logging.hpp"

namespace fsc {
bool UDEFactory::Register(std::string name, UDECreator creator) {
  auto [_, success] =
      registry_.try_emplace(std::move(name), std::move(creator));
  return success;
}

UDEFactory::UDEUniquePtr UDEFactory::Create(ParameterBaseSharedPtr params,
                                            LoggerBase* logger) {
  if (registry_.empty()) {
    if (logger) {
      logger->log(Severity::kError, "No UDE have been registered");
      return {};
    }
  }

  const auto& name = params->parameterFor();
  if (name.empty()) {
    if (logger) {
      logger->log(Severity::kError) << "UDE name is empty";
      return {};
    }
  }

  if (auto it = registry_.find(name); it != registry_.end()) {
    return it->second(std::move(params));
  }

  if (logger) {
    std::vector<std::string> ude_types(registry_.size());
    fsc::UDEFactory::GetRegistryKeys(ude_types.begin());
    std::ostringstream oss;
    oss << ude_types.front();
    for (auto it = std::next(ude_types.begin()); it != ude_types.end(); ++it) {
      oss << ", " << *it;
    }
    logger->log(Severity::kError)
        << "Failed to create [" << name
        << "]: Not a UDE. Available UDE types are: " << oss.str();
  }
  return {};
}
}  // namespace fsc
