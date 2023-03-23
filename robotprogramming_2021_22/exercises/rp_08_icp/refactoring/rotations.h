#pragma once
#include <Eigen/Core>

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> Rx(Scalar theta) {
  Scalar c=cos(theta);
  Scalar s=sin(theta);
  Eigen::Matrix<Scalar, 3, 3> R;
  R <<
    1,  0,  0,
    0,  c, -s,
    0,  s,  c;
  return R;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> Ry(Scalar theta) {
  Scalar c=cos(theta);
  Scalar s=sin(theta);
  Eigen::Matrix<Scalar, 3, 3> R;
  R <<
    c,  0,  s,
    0,  1,  0,
   -s,  0,  c;
  return R;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> Rz(Scalar theta) {
  Scalar c=cos(theta);
  Scalar s=sin(theta);
  Eigen::Matrix<Scalar, 3, 3> R;
  R <<
    c, -s, 0,
    s,  c, 0,
    0,  0, 1;
  return R;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 2, 2> Rtheta(Scalar theta) {
  Scalar c=cos(theta);
  Scalar s=sin(theta);
  Eigen::Matrix<Scalar, 2, 2> R;
  R <<
    c, -s, 
    s,  c;
  return R;
}

