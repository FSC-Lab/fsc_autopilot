#include "utils/utils.hpp"
#include <math.h>

namespace utils {
    double SatSmooth0(double input, double c)
    {
        return input / sqrt(c + input * input);
    }
    double SatHard0(double input, double upper, double lower)
    {
        if (input > upper) {
            return upper;
        } else if (input < lower) {
            return lower;
        } else {
            return input;
        }
    }
}