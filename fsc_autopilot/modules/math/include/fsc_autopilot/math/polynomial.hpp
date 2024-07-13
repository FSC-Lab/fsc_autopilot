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

#ifndef FSC_AUTOPILOT_MATH_POLYNOMIAL_HPP_
#define FSC_AUTOPILOT_MATH_POLYNOMIAL_HPP_

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

#endif  // FSC_AUTOPILOT_MATH_POLYNOMIAL_HPP_
