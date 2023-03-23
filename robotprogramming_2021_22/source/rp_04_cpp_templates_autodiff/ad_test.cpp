#include <iostream>
#include "ad.h"

using namespace std;

float f(const float& x, bool value_false_derivative_true=false) {
  // we cache the constants
  static const DualValuef _5=DualValuef(5);
  static const DualValuef _3=DualValuef(3);
  
  // convert to DualValue the linearization point;
  DualValuef _x(x);

  // if we want to compute the derivative, we need to set
  // to one the x' value on the item
  if (value_false_derivative_true)
    _x.derivative=1.f;
  // here we go with the calculation
  const DualValuef _y=cos(_5*_x)/(sin(exp(_x+_3)));

  // depending on what is asked we return either the value field
  // or the derivative field
  if (value_false_derivative_true) {
    return _y.derivative;
  } else
    return _y.value;
}

int main() {
  float x=0.5;
  float y=f(x);
  float dy_dx=f(x,true);
  cout << "the value of f in " << x << " is: " << y << "its derivative is: " << dy_dx << endl;
  return 0;
}
