#include "vec_template.h"

#include <iostream>
#include <assert.h>

using namespace std;

// read/write access to element at pos

template <typename Scalar_, int Dim_>
Scalar_& VecTemplate_<Scalar_, Dim_>::at(int pos){
  return v[pos];
}
  
// read access to element at pos
// const after () means that the method does not modify the invoking object
template <typename Scalar_, int Dim_>
const Scalar_& VecTemplate_<Scalar_, Dim_>::at(int pos) const {
  return v[pos];
}


//returns the sum this + other
template <typename Scalar_, int Dim_>
VecTemplate_<Scalar_, Dim_> VecTemplate_<Scalar_, Dim_>::operator + (const VecTemplate_<Scalar_, Dim_>& other) const {
  VecTemplate_ returned(*this);
  for (int i=0; i<Dim_; ++i)
    returned.v[i]+=other.v[i];
  return returned;
}

template <typename Scalar_, int Dim_>
VecTemplate_<Scalar_, Dim_> VecTemplate_<Scalar_, Dim_>::operator - (const VecTemplate_<Scalar_, Dim_>& other) const {
  VecTemplate_ returned(*this);
  for (int i=0; i<Dim_; ++i)
    returned.v[i]-=other.v[i];
  return returned;
}


// returns this*f
template <typename Scalar_, int Dim_>
VecTemplate_<Scalar_, Dim_>  VecTemplate_<Scalar_, Dim_>::operator* (Scalar_ f) const {
  VecTemplate_ returned(*this);
  for (int i=0; i<Dim_; ++i)
    returned.v[i]*=f;
  return returned;
}


// returns the dot product (vecs should have the same size);
template <typename Scalar_, int Dim_>
Scalar_ VecTemplate_<Scalar_, Dim_>::operator*(const VecTemplate_<Scalar_, Dim_>& other) const {
  Scalar_ acc=0.f;
  for (int i=0; i<Dim_; ++i)
    acc+=v[i]*other.v[i];
  return acc;
}

template <typename Scalar_, int Dim_>
ostream& operator << (ostream& os, const VecTemplate_<Scalar_, Dim_>& v) {
  os << "{ ptr: " << &v << " dim:" << v.Dim << " vals: [ ";
  for (int i=0; i<v.Dim; ++i)
    os << v.at(i) <<  " ";
  os << "] }";
  return os;
}
