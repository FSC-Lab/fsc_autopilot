#ifndef FSC_AUTOPILOT_ESTIMATOR_ESTIMATOR_BASE_HPP_
#define FSC_AUTOPILOT_ESTIMATOR_ESTIMATOR_BASE_HPP_

#include "Eigen/Dense"

namespace fsc {

class EstimatorBase {
 public:
  using VectorRef = Eigen::Ref<Eigen::VectorXd>;
  using VectorConstRef = Eigen::Ref<const Eigen::VectorXd>;

  virtual ~EstimatorBase() = default;

  virtual bool timeUpdate(const VectorConstRef& input, double dt);

  virtual bool measurementUpdate(const VectorConstRef& measurement,
                                 const VectorConstRef& online_data);

  virtual bool getEstimate(VectorRef estimate) = 0;
  virtual bool resetEstimate(VectorConstRef estimate) = 0;

  [[nodiscard]] virtual int numStates() const noexcept = 0;
  [[nodiscard]] virtual int numErrorStates() const noexcept = 0;
  [[nodiscard]] virtual int numInputs() const noexcept = 0;
  [[nodiscard]] virtual int numMeasurements() const noexcept = 0;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ESTIMATOR_ESTIMATOR_BASE_HPP_
