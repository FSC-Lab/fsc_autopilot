
#include "tracking_control/ros_support.hpp"

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
