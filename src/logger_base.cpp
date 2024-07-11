#include "tracking_control/core/logger_base.hpp"

namespace fsc::details {

LoggerStream::~LoggerStream() {
  const auto& message = stream_.str();
  if (!message.empty()) {
    logger_.log(severity_, message.c_str());
  }
}
}  // namespace fsc::details
