#ifndef FSC_AUTOPILOT_MATH_LOW_PASS_FILTER_HPP_
#define FSC_AUTOPILOT_MATH_LOW_PASS_FILTER_HPP_
#include <cmath>

#include "Eigen/Dense"
#include "fsc_autopilot/math/numbers.hpp"
#include "fsc_autopilot/utils/asserts.hpp"

namespace fsc {
template <typename Scalar>
class LowPassFilter {
 public:
  LowPassFilter(Scalar cutoff, Scalar dt)
      : cutoff_(cutoff),
        alpha_(Scalar{1} -
               std::exp(-dt * Scalar{2} * numbers::pi_v<Scalar> * cutoff_)) {
    Expects(cutoff > Scalar{0} && dt > Scalar{0});
  }

  Scalar update(Scalar input) { return output_ += (input - output_) * alpha_; }

  Scalar update(Scalar input, Scalar dt, Scalar cutoff = Scalar{-1}) {
    reconfigureFilter(dt, cutoff);  // Changes epow_ accordingly.
    return update(input);
  }

  void reconfigureFilter(Scalar dt, Scalar cutoff = Scalar{-1}) {
    // dt must be positive
    Expects(dt > Scalar{0});

    // If cutoff frequency is negative, then keep the last cutoff frequency
    if (cutoff > Scalar{0}) {
      cutoff_ = cutoff;
    }

    alpha_ =
        Scalar{1} - std::exp(-dt * Scalar{2} * numbers::pi_v<Scalar> * cutoff_);
  }

 private:
  Scalar cutoff_;
  Scalar alpha_;
  Scalar output_;
};

template <typename Matrix, typename Scalar = typename Matrix::Scalar>
class BatchLowPassFilter {
 public:
  using MatrixType = Matrix;

  BatchLowPassFilter() = default;

  template <typename Derived>
  BatchLowPassFilter(
      Scalar cutoff, Scalar dt,
      const Eigen::MatrixBase<Derived>& init_output = Matrix::Zero())
      : cutoff_(cutoff),
        alpha_(Scalar{1} -
               std::exp(-dt * Scalar{2} * numbers::pi_v<Scalar> * cutoff_)),
        output_(init_output) {
    Expects(cutoff > Scalar{0} && dt > Scalar{0});
  }

  template <typename Derived>
  Matrix update(const Eigen::MatrixBase<Derived>& input) {
    return output_ += (input - output_) * alpha_;
  }

  template <typename Derived>
  Matrix update(const Eigen::MatrixBase<Derived>& input, Scalar dt,
                Scalar cutoff = Scalar{-1}) {
    reconfigureFilter(dt, cutoff);  // Changes epow_ accordingly.
    return update(input);
  }

  void reconfigureFilter(Scalar dt, Scalar cutoff = Scalar{-1}) {
    // dt must be positive
    Expects(dt > Scalar{0});

    // If cutoff frequency is negative, then keep the last cutoff frequency
    if (cutoff > Scalar{0}) {
      cutoff_ = cutoff;
    }

    alpha_ =
        Scalar{1} - std::exp(-dt * Scalar{2} * numbers::pi_v<Scalar> * cutoff_);
  }

 private:
  Scalar cutoff_;
  Scalar alpha_;
  Matrix output_;
};
}  // namespace fsc

#endif  // FSC_AUTOPILOT_MATH_LOW_PASS_FILTER_HPP_
