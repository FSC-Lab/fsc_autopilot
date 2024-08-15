
#include "fsc_autopilot/estimator/ins10.hpp"

#include "fsc_autopilot/core/meta.hpp"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/math/rotation.hpp"

#define FAIL_UNLESS_FINITE(mat) \
  do {                          \
    if (!(mat).allFinite()) {   \
      return false;             \
    }                           \
  } while (0)

namespace fsc {

bool INS10::timeUpdate(const VectorConstRef& input, double dt) {
  Eigen::Ref<const Eigen::Vector3d> position = state_.head<3>();
  Eigen::Map<const Eigen::Quaterniond> orientation{state_.segment<4>(3).data()};
  Eigen::Ref<const Eigen::Vector3d> velocity = state_.tail<3>();

  if (input.size() != numInputs()) {
    return false;
  }

  Eigen::Ref<const Eigen::Vector3d> acceleration = input.head<3>();
  Eigen::Ref<const Eigen::Vector3d> body_rates = input.tail<3>();

  state_.head<3>() = position + dt * velocity;
  state_.segment<4>(3) =
      (orientation * AngleAxisToQuaternion(body_rates * dt)).coeffs();
  state_.tail<3>() =
      velocity + dt * (orientation * acceleration -
                       numbers::std_gravity * Eigen::Vector3d::UnitZ());

  // NOLINTBEGIN
  constexpr auto I = LIFT(Eigen::Matrix3d::Identity);
  constexpr auto O = LIFT(Eigen::Matrix3d::Zero);
  StateJacobian Fm = StateJacobian::Zero();
  InputJacobian Gm = InputJacobian::Zero();
  // NOLINTEND

  const Eigen::Matrix3d rotation_matrix = orientation.toRotationMatrix();
  const Eigen::Matrix3d dphi_by_phi =
      AngleAxisToRotationMatrix(-dt * body_rates);
  const Eigen::Matrix3d dv_by_phi = -dt * rotation_matrix * hat(acceleration);

  Fm << I(), O(), dt * I(),   //
      O(), dphi_by_phi, O(),  //
      O(), dv_by_phi, I();
  FAIL_UNLESS_FINITE(Fm);

  Gm << O(), O(),     //
      O(), dt * I(),  //
      dt * rotation_matrix, O();
  FAIL_UNLESS_FINITE(Gm);

  const StateJacobian new_covariance =
      Fm * cov_ * Fm.transpose() + Gm * input_cov_ * Gm.transpose();
  FAIL_UNLESS_FINITE(new_covariance);
  cov_ = new_covariance;
  return true;
}

bool INS10::measurementUpdate(const VectorConstRef& measurement,
                              const VectorConstRef& online_data) {
  if (measurement.size() != numMeasurements()) {
    return false;
  }
  Eigen::Ref<const Eigen::Vector3d> gps_position = measurement;

  Eigen::Vector3d innov = measurement - state_.head<3>();

  // NOLINTBEGIN
  constexpr auto I = LIFT(Eigen::Matrix3d::Identity);
  constexpr auto O = LIFT(Eigen::Matrix3d::Zero);
  MeasurementJacobian Hm = MeasurementJacobian::Zero();
  // NOLINTEND

  Hm << I(), O(), O();
  const Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> residual_cov =
      Hm * cov_ * Hm.transpose() + meas_cov_;

  const Eigen::Matrix<double, kErrorStateSize, kMeasurementSize> kalman_gain =
      residual_cov.transpose().llt().solve(Hm * cov_).transpose();
  FAIL_UNLESS_FINITE(kalman_gain);

  const Eigen::Matrix<double, kErrorStateSize, 1> state_update =
      kalman_gain * innov;

  state_.head<3>() += state_update.head<3>();
  state_.segment<4>(3) +=
      AngleAxisToQuaternion(state_update.segment<3>(3)).coeffs();
  state_.tail<3>() += state_update.tail<3>();
  return true;
}

bool INS10::getEstimate(VectorRef estimate) {
  if (estimate.size() != kStateSize) {
    return false;
  }
  estimate = state_;
  return true;
}

bool INS10::resetEstimate(const VectorConstRef& state) {
  if (state_.size() != kStateSize) {
    return false;
  }
  if (!state_.allFinite()) {
    return false;
  }
  state_ = state;
  return true;
}

}  // namespace fsc
