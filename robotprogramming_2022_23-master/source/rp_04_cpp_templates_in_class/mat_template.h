#pragma once
#include "vec_template.h"

template <typename Scalar_, int Rows_, int Cols_>
struct MatTemplate_ {
  static constexpr int Rows=Rows_;
  static constexpr int Cols=Cols_;
  static constexpr int Dim=Rows*Cols;
  using Scalar = Scalar_;
  using ThisType = MatTemplate_<Scalar, Rows, Cols>;
  using ThisTypeTransposed = MatTemplate_<Scalar, Cols, Rows>;
  using ColVectorType = VecTemplate_<Scalar, Rows>;
  using RowVectorType = VecTemplate_<Scalar, Cols>;
  
  Scalar_ v[Dim];
 
   // fills the matrix with f;
  void fill(Scalar f) {
    for (int i=0; i<Dim; ++i)
      v[i]=f;
  }

  //fills with random values;
  void randFill() {
    for (int i=0; i<Dim; ++i) {
      Scalar random_value=drand48();
      v[i]=random_value;
    }
  }
  
  // read/write access to element at pos
  Scalar& at(int pos) {
    return v[pos];
  }
  
  // read access to element at pos
  // const after () means that the method does not modify the invoking object
  const Scalar& at(int pos) const {
    return v[pos];
  }

    // read/write access to element at pos
  Scalar& at(int r, int c) {
    return v[r*Cols+c];
  }
  
  // read access to element at pos
  // const after () means that the method does not modify the invoking object
  const Scalar& at(int r, int c) const {
    return v[r*Cols+c];
  }

  //returns the sum this + other
  ThisType operator + (const ThisType& other) const {
    ThisType returned = *this;
    for (int i=0; i<Dim; ++i) {
      returned.v[i] += other.v[i];
    }
    return returned;
  }

  //returns the difference this - other
  ThisType operator - (const ThisType& other) const {
    ThisType returned = *this;
    for (int i=0; i<Dim; ++i) {
      returned.v[i] -= other.v[i];
    }
    return returned;
  }

  // returns this*f
  ThisType  operator* (Scalar f) const {
    ThisType returned = *this;
    for (int i=0; i<Dim; ++i) {
      returned.v[i] *= f;
    }
    return returned;
  }

  // returns this* other
  ColVectorType  operator* (const RowVectorType& other) const {
    ColVectorType result;
    result.fill(0);
    for (int r=0; r<Rows; ++r) {
      for (int c=0; c<Cols; ++c) {
        result.at(r) += at(r,c)*other.at(c);
      }
    }
    return result;
  }


  // returns this* other
  template <int OtherCols_>
  MatTemplate_<Scalar, Rows, OtherCols_> operator* (const MatTemplate_<Scalar, Cols, OtherCols_> & other) const {
    MatTemplate_<Scalar, Rows, OtherCols_> returned;
    returned.fill(Scalar(0));
    for (int r=0; r<Rows; ++r)
      for (int c=0; c<OtherCols_; ++c)
        for (int cc=0; cc<Cols; ++cc) {
          returned.at(r,c) += at(r,cc)*other.at(cc,c);
        }
    return returned;
  }

  ThisTypeTransposed transpose() const {
    ThisTypeTransposed result;
    for (int r=0; r<Rows; ++r) {
      for (int c=0; c<Cols; ++c) {
        result.at(c,r) = at(r,c);
      }
    }
    return result;
  }
};

template <typename Scalar_, int Rows_, int Cols_>
std::ostream& operator << (std::ostream& os, const MatTemplate_<Scalar_, Rows_, Cols_>& m) {
  for (int r=0; r<m.Rows; ++r) {
    for (int c=0; c<m.Cols; ++c)
      os << m.at(r,c) << " ";
    os << endl;
  }
  return os;
}
