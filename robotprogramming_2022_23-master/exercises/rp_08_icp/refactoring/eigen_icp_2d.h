#pragma once
#include "eigen_icp.h"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>

template <typename ContainerType>
class ICP_2D_: public ICP_<ContainerType>{
public:
  using ICPBase   = ICP_<ContainerType>;
  using PointType = typename ICPBase::PointType;
  using Scalar    = typename ICPBase::Scalar;
  using TransformType = typename ICPBase::TransformType;
  ICP_2D_ (const ContainerType& fixed_,
           const ContainerType& moving_,
           int min_points_in_leaf):
    ICPBase(fixed_, moving_, min_points_in_leaf)
  {}

  void optimizeCorrespondences() {
    Eigen::Matrix<Scalar, 3, 1> dx;
    Eigen::Matrix<Scalar, 3, 3> H;
    Eigen::Matrix<Scalar, 3, 1> b;
    H.setZero();
    b.setZero();
    Eigen::Matrix<Scalar, 2, 3> J;
    J.setIdentity();
    this->_num_kernelized=0;
    this->_num_inliers=0;
    this->_chi2_sum=0;
    for (const auto& c: this->_correspondences) {
      const auto& f=c._fixed;
      const auto& m=c._moving;
      J(0,2) = -m.y();
      J(1,2) = m.x();
      PointType e=m-f;
      Scalar scale=1;
      Scalar chi=e.squaredNorm();
      this->_chi2_sum+=chi;
      if (e.squaredNorm()>this->_kernel_chi2) {
        scale=sqrt(this->_kernel_chi2/chi);
        ++this->_num_kernelized;
      } else {
        ++this->_num_inliers;
      }
      H.noalias()+= scale* J.transpose()*J;
      b.noalias()+= scale* J.transpose()*e;
    }
    dx=H.ldlt().solve(-b);
    TransformType dX;
    const Eigen::Matrix<Scalar, 2, 2> dR=Rtheta(dx(2));
    dX.setIdentity();
    dX.linear()=dR;
    dX.translation()=dx.block(0,0,2,1);
    this->_X=dX*this->_X;
  }
};
