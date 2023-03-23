#pragma once
#include <iostream>

template <typename Scalar_, int Dim_ > 
struct VecTemplate_ {
  Scalar_ v[Dim_]; // elements to the data
  static const int Dim = Dim_;
  using Scalar = Scalar_;
  
  // read/write access to element at pos
  Scalar_& at(int pos);

  void fill(const Scalar& s) {
    for (int i=0; i<Dim; ++i)
      v[i]=s;
  }
  
  // read access to element at pos
  // const after () means that the method does not modify the invoking object
  const Scalar_& at(int pos) const;

  //returns the sum this + other
  VecTemplate_ operator + (const VecTemplate_& other) const;

  //returns the difference this - other
  VecTemplate_ operator - (const VecTemplate_& other) const;

  // returns this*f
  VecTemplate_  operator* (Scalar_ f) const;

  // returns the dot product (vecs should have the same size);
  Scalar_ operator*(const VecTemplate_& other) const;
  
};


template <typename Scalar_, int Dim_>
std::ostream& operator << (std::ostream& os, const VecTemplate_<Scalar_, Dim_>& src);

#include "vec_template_impl.cpp"
