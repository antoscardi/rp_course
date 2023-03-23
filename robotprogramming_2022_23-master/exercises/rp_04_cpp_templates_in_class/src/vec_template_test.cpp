#include <iostream>
#include "vec_template.h"

using namespace std;

using MyVecType=VecTemplate_<float, 3>;

int main() {
  MyVecType  v1;
  for (int i=0; i<v1.Dim; ++i)
    v1.at(i)=i;

  cerr << "v1: " << v1 << endl;

  MyVecType v2(v1);
  cerr << "v2: " << v2 << endl;

  MyVecType v3;
  v3=v2;
  cerr << "v3: " << v3 << endl;

  __asm__ ("#begin stuff");
  MyVecType v45=v1;
  for (int i=0; i<45; ++i) {
    v45=v45+v1;
  }
  __asm__ ("#end stuff");
  
  cerr << "v45: " << v45 << endl;
  
  cerr << "sum: " << v1+v2 << endl;

  cerr << "diff: " << v1-v2 << endl;

  v1.at(2)+=4;
  cerr << v1+(v2*2.f) << endl;
}
