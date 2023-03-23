#include <iostream>
#include "mat_template.h"
using namespace std;

using Scalar=float;
using Vec3=VecTemplate_<Scalar,3>;
using Mat3=MatTemplate_<Scalar,3,3>;
using Mat3_4=MatTemplate_<Scalar,3,4>;
using Mat2_3=MatTemplate_<Scalar,2,3>;
using Mat3_2=MatTemplate_<Scalar,3,2>;
using Mat3_1=MatTemplate_<Scalar,3,1>;

  

int main() {
  Vec3 v1;
  v1.at(0)=1;
  v1.at(1)=2;
  v1.at(2)=3;
  cout << v1 << endl;

  Mat3 m;
  m.fill(0.);
  cout << m;
  Mat3 m2;
  m2.fill(0);
  m2.at(0,0)=1;
  m2.at(1,1)=2;
  m2.at(2,2)=3;

  cout << m2 << endl;
  cout << m + m2 << endl;
  Mat3 m3=m + m2;
 
  cout << m + m2 << endl;

  cout  << m3-m2 << endl;
  cout  << m * v1 << endl;
  cout  << m3 * v1 << endl;

  Mat3_4 m4;
  cerr << "m4 " << m4.Dim << endl;
  m4.fill(1);
  cout << m4 << endl;
  
  cout << (m3*m4).transpose() << endl;

  Mat3_4 m5;
  m5.randFill();
  cout << m5 << endl;
  
  cout << m5*m5.transpose() << endl;

  m.randFill();
  cout << m << endl;

  cout << m-m5*m5.transpose() << endl;
  cout << "DONE" << endl;



    //cout << m4*m4 << endl; << compile time error
  Mat3 A;
  A.randFill();

  Mat3_2 B;
  B.randFill();

  Mat2_3 C;
  C.randFill();
  
  Mat3_1 v;
  v.randFill();
  for (int i=0; i<10000000; ++i) {
    auto result=A*(B*C+v*v.transpose());
  }

}
