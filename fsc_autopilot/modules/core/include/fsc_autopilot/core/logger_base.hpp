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

#ifndef FSC_AUTOPILOT_CORE_LOGGER_BASE_HPP_
#define FSC_AUTOPILOT_CORE_LOGGER_BASE_HPP_

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

#endif  // FSC_AUTOPILOT_CORE_LOGGER_BASE_HPP_
