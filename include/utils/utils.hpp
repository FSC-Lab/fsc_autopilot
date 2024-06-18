#ifndef UTILS_HPP
#define UTILS_HPP
#include "Eigen/Dense"

namespace utils
{
    // saturation functions
    double SatSmooth0(double input, double c);
    double SatHard0(double input, double upper, double lower);
}; // namespace name

#endif