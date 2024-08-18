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
#include "fsc_autopilot/position_control/position_controller_factory.hpp"

#include "fsc_autopilot/core/logger_base.hpp"

namespace fsc {
bool PositionControllerFactory::Register(std::string name, Creator creator) {
  auto [_, success] =
      registry_.try_emplace(std::move(name), std::move(creator));
  return success;
}

PositionControllerFactory::ProductUniquePtr PositionControllerFactory::Create(
    const std::string& name, LoggerBase& logger) {
  if (registry_.empty()) {
    logger.log(Severity::kError, "No PositionController have been registered");
    return {};
  }

  if (name.empty()) {
    logger.log(Severity::kError) << "PositionController name is empty";
    return {};
  }

  if (auto it = registry_.find(name); it != registry_.end()) {
    logger.log(Severity::kInfo)
        << "Creating `" << name << "` PositionController";

    return it->second();
  }

  std::vector<std::string> ude_types(registry_.size());
  fsc::PositionControllerFactory::GetRegistryKeys(ude_types.begin());
  std::ostringstream oss;
  oss << ude_types.front();
  for (auto it = std::next(ude_types.begin()); it != ude_types.end(); ++it) {
    oss << ", " << *it;
  }
  logger.log(Severity::kError) << "Failed to create [" << name
                               << "]: Not a PositionController. Available "
                                  "PositionController types are: "
                               << oss.str();

  return {};
}
}  // namespace fsc
