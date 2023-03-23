#include <iostream>
#include "ad.h"

using namespace std;

DualValuef f(const DualValuef& x,
             const DualValuef& y) {
  // we cache the constants
  static const DualValuef _2=DualValuef(2);
  return (sin(x)*cos(y))/log(_2+sin(x));
}

int main() {
  DualValuef x(-1.);
  const float increment = 0.1;
  cout << "# x y f(x,y) df/dx df/dy" << endl;
  while(x.value<1.) {
    x.value+=increment;
    DualValuef y(-1);
    while(y.value<1.){
      y.value+=increment;
      y.derivative=0;
      x.derivative=1;

      DualValuef z=f(x,y);
      cout << x.value << " " << y.value << " " << z.value << " " << z.derivative;

      x.derivative=0;
      y.derivative=1;
      z=f(x,y);
      cout << " " << z.derivative << endl;
    }
  }
  
  return 0;
}
