#ifndef TRACKING_CONTROL_ROS_SUPPORT_HPP_
#define TRACKING_CONTROL_ROS_SUPPORT_HPP_

#include <memory>
#include <string>

#include "fsc_autopilot/core/logger_base.hpp"
#include "fsc_autopilot/core/parameter_base.hpp"
#include "ros/console.h"
#include "ros/node_handle.h"

namespace nodelib {
class RosLogger : public fsc::LoggerBase {
 public:
  using RosSeverity = ros::console::levels::Level;

  explicit RosLogger(std::string name);

  void log(fsc::Severity severity, const char* msg) noexcept override;

  [[nodiscard]] std::string name() const override { return name_; }

 private:
  std::string name_;
};

class RosParamLoader : public fsc::ParameterLoaderBase {
 public:
  explicit RosParamLoader(const ros::NodeHandle& pnh);
  explicit RosParamLoader(const std::string& pnh);

  [[nodiscard]] std::shared_ptr<fsc::ParameterLoaderBase> getChildLoader(
      const std::string& ns) const override;

  bool getParam(const std::string& key, bool& value) const override;

  bool getParam(const std::string& key, int& value) const override;

  bool getParam(const std::string& key, double& value) const override;

  bool getParam(const std::string& key, std::string& value) const override;

 private:
  ros::NodeHandle pnh_;
};

}  // namespace nodelib

#endif  // TRACKING_CONTROL_ROS_SUPPORT_HPP_
