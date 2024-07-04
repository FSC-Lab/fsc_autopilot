#ifndef TRACKING_CONTROL_LOGGING_HPP_
#define TRACKING_CONTROL_LOGGING_HPP_

#include <cstdint>
#include <sstream>
#include <string>

namespace fsc {

class LoggerBase;
enum class Severity : std::int32_t {
  kInternalError = 0,
  kError = 1,
  kWarning = 2,
  kInfo = 3,
  kVerbose = 4,
};

namespace details {

class LoggerStream {
 public:
  ~LoggerStream();

  template <typename T>
  LoggerStream& operator<<(const T& value) {
    stream_ << value;
    return (*this);
  }

  friend class fsc::LoggerBase;

 private:
  LoggerStream(LoggerBase& logger, Severity severity)
      : logger_(logger), severity_(severity) {}

  LoggerBase& logger_;
  Severity severity_;
  std::ostringstream stream_;
};
}  // namespace details

class LoggerBase {
 public:
  struct SourceLocation {
    int line;
    const char* file;
  };

  virtual void log(Severity severity, char const* msg) noexcept = 0;

  details::LoggerStream log(Severity severity) { return {*this, severity}; }

  LoggerBase() = default;
  virtual ~LoggerBase() = default;

  [[nodiscard]] virtual std::string name() const { return ""; }

 protected:
  LoggerBase(LoggerBase const&) = default;
  LoggerBase(LoggerBase&&) = default;
  LoggerBase& operator=(LoggerBase const&) & = default;
  LoggerBase& operator=(LoggerBase&&) & = default;
};
}  // namespace fsc

#endif  // TRACKING_CONTROL_LOGGING_HPP_
