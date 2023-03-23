#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"

using Vector3f = Eigen::Vector3f;

class ICP {
protected:
  struct PointPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointPair(const Vector3f& fixed_, const Vector3f& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointPair(){}
    Vector3f _fixed;
    Vector3f _moving;
  };
  using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair>>;

public:
  using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;

  ICP(const ContainerType& fixed_,
      const ContainerType& moving_,
      int min_points_in_leaf);

  void computeCorrespondences();
  
  // for me to test
  void computeCorrespondencesFake();
  
  void optimizeCorrespondences();
  
  void run(int max_iterations);

  const Eigen::Isometry3f& X() const {return _X;}
  Eigen::Isometry3f& X()  {return _X;}
  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
  inline const Eigen::Matrix<float, 6,1>& dx() const {return _dx;}
 
protected:
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  ContainerType _fixed;
  const ContainerType& _moving;
  Eigen::Isometry3f _X=Eigen::Isometry3f::Identity();
  TreeNodeType _kd_tree;
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  PointPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 6,1> _dx;
};
