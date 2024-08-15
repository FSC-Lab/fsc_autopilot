#ifndef FSC_AUTOPILOT_ESTIMATOR_INS10_HPP_
#define FSC_AUTOPILOT_ESTIMATOR_INS10_HPP_

#include "Eigen/Dense"
#include "fsc_autopilot/core/definitions.hpp"
#include "fsc_autopilot/core/vehicle_input.hpp"
#include "fsc_autopilot/estimator/estimator_base.hpp"

namespace fsc {
class INS10 : public EstimatorBase {
 public:
  enum {
    kStateSize = 10,
    kErrorStateSize = 9,
    kInputSize = 6,
    kMeasurementSize = 3
  };

  template <typename Derived>
  explicit INS10(const Eigen::MatrixBase<Derived>& state) : state_(state) {}

  using StateType = Eigen::Matrix<double, kStateSize, 1>;
  using StateJacobian = Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>;

  using InputJacobian = Eigen::Matrix<double, kErrorStateSize, kInputSize>;
  using InputCovariance = Eigen::Matrix<double, kInputSize, kInputSize>;

  using MeasurementJacobian =
      Eigen::Matrix<double, kMeasurementSize, kErrorStateSize>;
  using MeasurementCovariance =
      Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  bool timeUpdate(const VectorConstRef& input, double dt) override;

  bool measurementUpdate(const VectorConstRef& measurement,
                         const VectorConstRef& online_data) override;

  bool getEstimate(VectorRef estimate) override;

  bool resetEstimate(const VectorConstRef& state);

  [[nodiscard]] int numStates() const noexcept override { return kStateSize; }
  [[nodiscard]] int numErrorStates() const noexcept override {
    return kErrorStateSize;
  }
  [[nodiscard]] int numInputs() const noexcept override { return kInputSize; }
  [[nodiscard]] int numMeasurements() const noexcept override {
    return kMeasurementSize;
  }

 private:
  StateType state_;
  StateJacobian cov_;
  InputCovariance input_cov_;
  MeasurementCovariance meas_cov_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_ESTIMATOR_INS10_HPP_
