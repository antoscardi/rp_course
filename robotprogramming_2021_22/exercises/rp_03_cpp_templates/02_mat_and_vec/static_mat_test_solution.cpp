#include <iostream>
#include "static_mat.h"
#include <unistd.h>
using namespace rp;
using namespace std;


int main() {
  Mat_<float, 3,3> A;
  randFill(A);
  cout << "A: " << A << endl;

  Mat_<float,3,2> B;
  randFill(B);
  cout << "B: " << B << endl;

  Mat_<float,2,3> C;
  randFill(C);
  cout << "C: " << C << endl;
  
  Mat_<float, 3,1> v;
  randFill(v);
  cout << "v: " << v << endl;

  Mat_<float, 3,3> result;
  for (int i=0; i<100000000; ++i) {
    result=A*(B*C+v*v.transpose());
  }
  cout << result;
}
