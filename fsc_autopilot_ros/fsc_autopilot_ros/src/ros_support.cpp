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

RosLogger::RosLogger(std::string name) : name_(std::move(name)) {}

void RosLogger::log(fsc::Severity severity, const char* msg) noexcept {
  switch (severity) {
    case fsc::Severity::kInternalError:
      ROS_FATAL("%s", msg);
      break;
    case fsc::Severity::kError:
      ROS_ERROR("%s", msg);
      break;
    case fsc::Severity::kWarning:
      ROS_WARN("%s", msg);
      break;
    case fsc::Severity::kInfo:
      ROS_INFO("%s", msg);
    case fsc::Severity::kVerbose:
      ROS_DEBUG("%s", msg);
      break;
  }
}

RosParamLoader::RosParamLoader(const ros::NodeHandle& pnh) : pnh_(pnh) {}

RosParamLoader::RosParamLoader(const std::string& pnh) : pnh_(pnh) {}

std::shared_ptr<fsc::ParameterLoaderBase> RosParamLoader::getChildLoader(
    const std::string& ns) const {
  return std::make_shared<RosParamLoader>(ros::NodeHandle(pnh_, ns));
}

bool RosParamLoader::getParam(const std::string& key, bool& value) const {
  return pnh_.getParam(key, value);
}

bool RosParamLoader::getParam(const std::string& key, int& value) const {
  return pnh_.getParam(key, value);
}

bool RosParamLoader::getParam(const std::string& key, double& value) const {
  return pnh_.getParam(key, value);
}

bool RosParamLoader::getParam(const std::string& key,
                              std::string& value) const {
  return pnh_.getParam(key, value);
}
}  // namespace nodelib
