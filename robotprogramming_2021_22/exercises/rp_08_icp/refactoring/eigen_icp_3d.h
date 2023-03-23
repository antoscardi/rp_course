#pragma once
#include "eigen_icp.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include <iostream>

template <typename ContainerType_>
class ICP_3D_: public ICP_<ContainerType_>{
public:
  using ICPBase=ICP_<ContainerType_>;
  using ContainerType = typename ICPBase::ContainerType;
  using PointType     = typename ICPBase::PointType;
  using Scalar        = typename ICPBase::Scalar;
  using TransformType = typename ICPBase::TransformType;
  ICP_3D_ (const ContainerType& fixed_,
           const ContainerType& moving_,
           int min_points_in_leaf):
    ICPBase(fixed_, moving_, min_points_in_leaf)
  {}

  void optimizeCorrespondences()  override {
    Eigen::Matrix<Scalar, 6,1> dx;
    Eigen::Matrix<Scalar, 6, 6> H;
    Eigen::Matrix<Scalar, 6, 1> b;
    H.setZero();
    b.setZero();
    Eigen::Matrix<Scalar, 3, 6> J;
    J.block(0,0,3,3).setIdentity();
    this->_num_kernelized=0;
    this->_num_inliers=0;
    this->_chi2_sum=0;
    for (const auto& c: this->_correspondences) {
      const auto& f=c._fixed;
      const auto& m=c._moving;
      J.block(0,3,3,3) <<
        0.f,   m.z(),    -m.y(),
        -m.z(),      0,   m.x(),
        m.y(),  -m.x(),   0;
      Eigen::Matrix<Scalar, ICPBase::Dim, 1> e=m-f;
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
    Eigen::Isometry3f dX;
    const Eigen::Matrix3f dR=Rx(dx(3))*Ry(dx(4))*Rz(dx(5));
    dX.setIdentity();
    dX.linear()=dR;
    dX.translation()=dx.block(0,0,3,1);
    this->_X=dX*this->_X;
  }
};

