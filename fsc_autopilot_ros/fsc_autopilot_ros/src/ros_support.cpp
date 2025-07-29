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

#include "fsc_autopilot_ros/ros_support.hpp"

namespace nodelib {

RosLogger::RosLogger(rclcpp::Logger logger) : logger_(logger) {}

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
      break;  // Added missing break statement
    case fsc::Severity::kVerbose:
      RCLCPP_DEBUG(logger_, "%s", msg);
      break;
  }
}

RosParamLoader::RosParamLoader(rclcpp::Node::SharedPtr node, const std::string& prefix) 
  : node_(node), prefix_(prefix) {}

std::shared_ptr<fsc::ParameterLoaderBase> RosParamLoader::getChildLoader(
    const std::string& ns) const {
  return std::make_shared<RosParamLoader>(node_, prefix_ + ns + ".");
}

bool RosParamLoader::getParam(const std::string& key, bool& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_bool();
  return true;
}

bool RosParamLoader::getParam(const std::string& key, int& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_int();
  return true;
}

bool RosParamLoader::getParam(const std::string& key,
                              std::vector<int>& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_integer_array();
  return true;
}

bool RosParamLoader::getParam(const std::string& key, double& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_double();
  return true;
}

bool RosParamLoader::getParam(const std::string& key,
                              std::vector<double>& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_double_array();
  return true;
}

bool RosParamLoader::getParam(const std::string& key,
                              std::string& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_string();
  return true;
}

bool RosParamLoader::getParam(const std::string& key,
                              std::vector<std::string>& value) const {
  if (!node_->has_parameter(prefix_ + key)) {
    return false;
  }
  value = node_->get_parameter(prefix_ + key).as_string_array();
  return true;
}

}  // namespace nodelib