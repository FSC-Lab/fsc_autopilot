#ifndef GEOMETRIC_CONTROL_POLYNOMIAL_HPP_
#define GEOMETRIC_CONTROL_POLYNOMIAL_HPP_

#include <numeric>

#include "Eigen/Dense"

namespace math {

template <typename Scalar,
          typename CoefficientType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>
class Polynomial {
 public:
  using Coefficients = CoefficientType;
  Polynomial() = default;

  template <typename Derived>
  explicit Polynomial(const Eigen::MatrixBase<Derived>& coeffs)
      : coeffs_(coeffs), n_cfs_(coeffs.size()), power_seq_(n_cfs_) {
    std::iota(power_seq_.data(), power_seq_.data() + n_cfs_, Scalar(0));
  }

  Scalar vals(const Scalar& x) const {
    if (n_cfs_) {
      return coeffs_.matrix().dot(Coefficients::Constant(n_cfs_, x)
                                      .array()
                                      .pow(power_seq_.array())
                                      .matrix());
    }

    // Comply with behavior of np.polyval([], some_val)
    return Scalar(0);
  }

  const Coefficients& coeffs() const { return coeffs_; }

 private:
  Coefficients coeffs_;
  Eigen::Index n_cfs_{0};
  Coefficients power_seq_;
};
}  // namespace math

#endif  // GEOMETRIC_CONTROL_POLYNOMIAL_HPP_
