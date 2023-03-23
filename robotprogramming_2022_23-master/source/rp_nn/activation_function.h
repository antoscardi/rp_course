#pragma once
#include <cmath>

struct SigmoidActivationFunction {
  static constexpr const char* NAME="Sigmoid";
  static inline void compute(float& output,
                             float& d_output,
                             const float& input) {
    float e=exp(-input);
    output=2./(1+e)-1.;
    d_output=-2.*e/pow(1+e,2);
  }
};
