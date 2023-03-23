#include "eigen_icp.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include "Eigen/Cholesky"
#include <iostream>

using namespace std;

ICP::ICP(const ContainerType& fixed_,
         const ContainerType& moving_,
         int min_points_in_leaf):
  _fixed(fixed_),
  _moving(moving_),
  _kd_tree(_fixed.begin(), _fixed.end(), min_points_in_leaf){}

void ICP::computeCorrespondences() {
  _correspondences.resize(_moving.size());
  int k=0;
  for (const auto& m: _moving) {
    const auto& mt=_X*m;
    auto ft=_kd_tree.bestMatchFast(mt, _ball_radius);
    if (! ft)
      continue;
    _correspondences[k]._fixed=*ft;
    _correspondences[k]._moving=mt;
    ++k;
  }
  _correspondences.resize(k);
}
  
// for me to test
void ICP::computeCorrespondencesFake() {
  // for me to test optimizer
  _correspondences.resize(_moving.size());
  for (size_t i=0; i<_moving.size(); ++i) {
    _correspondences[i]._fixed=_fixed[i];
    _correspondences[i]._moving=_X*_fixed[i];
  }
}

void ICP::optimizeCorrespondences() {
  Eigen::Matrix<float, 6, 6> H;
  Eigen::Matrix<float, 6, 1> b;
  H.setZero();
  b.setZero();
  Eigen::Matrix<float, 3, 6> J;
  J.block<3,3>(0,0).setIdentity();
  _num_kernelized=0;
  _num_inliers=0;
  _chi2_sum=0;
  for (const auto& c: _correspondences) {
    const auto& f=c._fixed;
    const auto& m=c._moving;
    J.block<3,3>(0,3) <<
      0.f,   m.z(),    -m.y(),
      -m.z(),      0,   m.x(),
      m.y(),  -m.x(),   0;
    Vector3f e=m-f;
    float scale=1;
    float chi=e.squaredNorm();
    _chi2_sum+=chi;
    if (e.squaredNorm()>_kernel_chi2) {
      scale=sqrt(_kernel_chi2/chi);
      ++_num_kernelized;
    } else {
      ++_num_inliers;
    }
    H.noalias()+= scale* J.transpose()*J;
    b.noalias()+= scale* J.transpose()*e;
  }
  _dx=H.ldlt().solve(-b);
  Eigen::Isometry3f dX;
  const Eigen::Matrix3f dR=Rx(_dx(3))*Ry(_dx(4))*Rz(_dx(5));
  dX.setIdentity();
  dX.linear()=dR;
  dX.translation()=_dx.block<3,1>(0,0);
  _X=dX*_X;
}

void ICP::run(int max_iterations) {
  int current_iteration=0;
  while (current_iteration<max_iterations) {
    computeCorrespondences();
    optimizeCorrespondences();
    ++current_iteration;
    cerr << "Iteration: " << current_iteration;
    cerr << " corr: " << numCorrespondences();
    cerr << " inl: " << numInliers();
    cerr << " ker: " << numKernelized();
    cerr << " chi: " << _chi2_sum << endl;
  }
}

