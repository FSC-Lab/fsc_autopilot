#ifndef TRACKING_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
#define TRACKING_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_

#include "Eigen/Dense"
#include "tracking_control/controller_base.hpp"
#include "tracking_control/internal/internal.hpp"

namespace control {
template <typename Scalar>
class NonlinearGeometricController;
template <typename Scalar>
struct NonlinearGeometricControllerState {
  Quaternion<Scalar> attitude;
};
template <typename Scalar>
struct NonlinearGeometricControllerReference {
  Quaternion<Scalar> attitude;
};

template <typename Scalar>
struct NonlinearGeometricControllerError {
  Vector3<Scalar> attitude_error;
};

template <typename Scalar>
struct NonlinearGeometricControllerOutput {
  Vector3<Scalar> body_rate;
};

template <typename Scalar>
struct NonlinearGeometricControllerParameters {
  Scalar time_constant;
};

template <typename S>
struct ControllerTraits<NonlinearGeometricController<S>> {
  using Scalar = S;
  using State = NonlinearGeometricControllerState<Scalar>;
  using Reference = NonlinearGeometricControllerReference<Scalar>;
  using Error = NonlinearGeometricControllerError<Scalar>;
  using Output = NonlinearGeometricControllerOutput<Scalar>;
  using Parameters = NonlinearGeometricControllerParameters<Scalar>;
};

template <typename Scalar>
struct NonlinearGeometricController
    : public ControllerBase<NonlinearGeometricController<Scalar>> {
  using Base = ControllerBase<NonlinearGeometricController<Scalar>>;

  using Vector3Type = Vector3<Scalar>;
  using QuaternionType = Quaternion<Scalar>;
  using Matrix3Type = Matrix3<Scalar>;

  using State = typename Base::State;
  using Reference = typename Base::Reference;
  using Result = typename Base::Result;
  using Parameters = typename Base::Parameters;

  Result runImpl(const State& state, const Reference& refs, double dt,
                 bool intFlag) const {
    const auto rotmat = state.attitude.toRotationMatrix();
    const auto rotmat_sp = refs.attitude.toRotationMatrix();

    // e_r = 1 / 2 * (Rd.T * R - R.T * Rd)
    const auto attitude_error = details::vee(rotmat_sp.transpose() * rotmat -
                                             rotmat.transpose() * rotmat_sp) /
                                Scalar(2);

    const auto body_rate_sp =
        -Scalar(2) / params_.time_constant * attitude_error;
    return {true, body_rate_sp, attitude_error};
  }
  const Parameters& params() const { return params_; }

  Parameters& params() { return params_; }

 private:
  Parameters params_;
};
}  // namespace control

#endif  // TRACKING_CONTROL_NONLINEAR_GEOMETRIC_CONTROLLER_HPP_
