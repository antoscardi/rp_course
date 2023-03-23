#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"

using Vector3f = Eigen::Vector3f; 

struct PointPair{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointPair(const Vector3f& fixed_, const Vector3f& moving_):
    _fixed(fixed_),
    _moving(moving_){};
    
  PointPair(){}
  Vector3f _fixed;
  Vector3f _moving;
};

using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair> >;



class ICP {
public:
  using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;
  using TreeNode = TreeNode_< ContainerType::iterator >;
  using TransformType = Eigen::Isometry3f;
  
  ICP(const ContainerType& fixed_,
      const ContainerType& moving_,
      int min_points_in_leaf,
      const TransformType& X_ = TransformType::Identity() );
  
  void run(int max_iterations);

  inline const Eigen::Isometry3f& X() const {return _X;}

  inline int numCorrespondences() const {return _correspondences.size();}

  // for optimization ignore now
  inline int numKernelized() const {return _num_kernelized;}

  inline int numInliers() const {return _num_inliers;}

  inline const Eigen::Matrix<float, 6,1>& dx() const {return _dx;}

  
protected:

  void computeCorrespondences();
  
  void optimizeCorrespondences();
  
  
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  ContainerType _fixed;
  const ContainerType& _moving;
  TransformType _X;
  TreeNodeType _kd_tree;
  PointPairVector _correspondences;

  //additional parameters needed for the optimization, ignore
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 6,1> _dx;
};
