#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"

using Vector2f = Eigen::Vector2f;

class ICP {
public:
  using TransformType = Eigen::Isometry2f;
  
  //fillme, it is gonna be easy
protected:
  Isometry2f _X;
  float _chi2_sum=0;
  float _kernel_chi=1.f;
  Eigen::Vector3f _dx;
};
