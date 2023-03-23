#include <iostream>
using namespace std;

template <typename Scalar_, int Dim_> 
struct Vec_ {
    Scalar_ values[Dim_];
  
  Scalar_& at(int pos) {
    return values[pos];
  }

  const Scalar_& at(int pos) const {
    return values[pos];
  }
  
  
  Vec_<Scalar_, Dim_> operator + (const Vec_<Scalar_, Dim_>& other) const {
    Vec_<Scalar_, Dim_> returned=*this;
    for (int i=0; i<Dim_; ++i)
      returned.values[i]+=other.values[i];
    return returned;
  }

  Vec_<Scalar_, Dim_> operator - (const Vec_<Scalar_, Dim_>& other) const {
    Vec_<Scalar_, Dim_> returned=*this;
    for (int i=0; i<Dim_; ++i)
      returned.values[i]-=other.values[i];
    return returned;

  }

  Vec_<Scalar_, Dim_>  operator* (Scalar_ f) const {
    Vec_<Scalar_, Dim_> returned=*this;
    for (int i=0; i<Dim_; ++i)
      returned.values[i]*=f;
    return returned;
  }

  Scalar_ operator*(const Vec_<Scalar_, Dim_>& other) const {
    Scalar_ acc(0);
    for (int i=0; i<Dim_; ++i)
      acc+=values[i] * other.values[i];
    return acc;
  }
  
};

template <typename Scalar_, int Dim_>
ostream& operator << (ostream& os, const Vec_<Scalar_, Dim_>& v) {
  for (int i=0; i<Dim_; ++i) {
    os << v.at(i) << " ";
  }
  return os;
}

using Vec5 = Vec_<float, 5>;
using Vec4 = Vec_<float, 4>;

int main() {
  Vec5 v5;
  v5.at(0)=0.1;
  v5.at(1)=0.2;
  cout << v5 << endl;
  Vec4 v4 = v5;
}
