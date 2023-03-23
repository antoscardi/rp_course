#include <iostream>
#include "mat_f.h"
using namespace std;

int main() {
  MatF A(3,3);
  A.randFill();
  cout << "A: " << A << endl;

  MatF B(3,2);
  B.randFill();
  cout << "B: " << B << endl;

  MatF C(2,3);
  C.randFill();
  cout << "C: " << C << endl;
  
  MatF v(3,1);
  v.randFill();
  cout << "v: " << v << endl;

  MatF result;
  for (int i=0; i<100000000; ++i) {
    result=A*(B*C+v*v.transpose());
  }
  cout << result;

}
