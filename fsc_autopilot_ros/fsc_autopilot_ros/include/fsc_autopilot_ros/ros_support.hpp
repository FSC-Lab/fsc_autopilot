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

#ifndef FSC_AUTOPILOT_ROS_ROS_SUPPORT_HPP_
#define FSC_AUTOPILOT_ROS_ROS_SUPPORT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nodelib {

class RosLogger : public fsc::LoggerBase {
 public:
  explicit RosLogger(rclcpp::Logger logger);

  void log(fsc::Severity severity, const char* msg) noexcept override;

  [[nodiscard]] std::string name() const override;

  rclcpp::Logger get_logger() const { return logger_; }

 private:
  rclcpp::Logger logger_;
};

class RosParamLoader : public fsc::ParameterLoaderBase {
 public:
  explicit RosParamLoader(rclcpp::Node::SharedPtr node, const std::string& prefix = "");

  [[nodiscard]] std::shared_ptr<fsc::ParameterLoaderBase> getChildLoader(
      const std::string& ns) const override;

  bool getParam(const std::string& key, bool& value) const override;

  bool getParam(const std::string& key, int& value) const override;

  bool getParam(const std::string& key, std::vector<int>& value) const override;

  bool getParam(const std::string& key, double& value) const override;

  bool getParam(const std::string& key,
                std::vector<double>& value) const override;

  bool getParam(const std::string& key, std::string& value) const override;

  bool getParam(const std::string& key,
                std::vector<std::string>& value) const override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string prefix_;
};

}  

#endif  