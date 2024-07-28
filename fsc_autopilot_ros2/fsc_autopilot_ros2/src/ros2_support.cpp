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

#include "fsc_autopilot_ros2/ros2_support.hpp"

#include <utility>

namespace nodelib {

RosLogger::RosLogger(const rclcpp::Logger& logger) : logger_(logger) {}
RosLogger::RosLogger(const rclcpp::Node& node) : logger_(node.get_logger()) {}

void RosLogger::log(fsc::Severity severity, const char* msg) noexcept {
  switch (severity) {
    case fsc::Severity::kInternalError:
      RCLCPP_FATAL(logger_, "%s", msg);
      break;
    case fsc::Severity::kError:
      RCLCPP_ERROR(logger_, "%s", msg);
      break;
    case fsc::Severity::kWarning:
      RCLCPP_WARN(logger_, "%s", msg);
      break;
    case fsc::Severity::kInfo:
      RCLCPP_INFO(logger_, "%s", msg);
    case fsc::Severity::kVerbose:
      RCLCPP_DEBUG(logger_, "%s", msg);
      break;
  }
}

RosParamLoader::RosParamLoader(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {}

std::shared_ptr<fsc::ParameterLoaderBase> RosParamLoader::getChildLoader(
    const std::string& ns) const {
  return std::make_shared<RosParamLoader>(node_->create_sub_node(ns));
}

bool RosParamLoader::getParam(const std::string& key, bool& value) const {
  return node_->get_parameter(key, value);
}

bool RosParamLoader::getParam(const std::string& key, int& value) const {
  return node_->get_parameter(key, value);
}

bool RosParamLoader::getParam(const std::string& key, double& value) const {
  return node_->get_parameter(key, value);
}

bool RosParamLoader::getParam(const std::string& key,
                              std::string& value) const {
  return node_->get_parameter(key, value);
}
}  // namespace nodelib
