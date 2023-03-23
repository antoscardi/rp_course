#pragma once
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "eigen_kdtree.h"
#include <iostream>

template <typename ContainerType_,
          Eigen::TransformTraits Mode_= Eigen::TransformTraits::Isometry>
class ICP_ {
protected:
  using ContainerType = ContainerType_;
  using IteratorType  = typename ContainerType::iterator;
  using PointType     = typename ContainerType::value_type;
  using Scalar        = typename PointType::Scalar;
  static constexpr int Dim=PointType::RowsAtCompileTime;
  using TransformType = Eigen::Transform<Scalar, Dim, Mode_>;
  struct PointPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointPair(const PointType& fixed_, const PointType& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointPair(){}
    PointType _fixed;
    PointType _moving;
  };
  using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair>>;
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  
  ContainerType _fixed;
  const ContainerType& _moving;
  TransformType _X =TransformType::Identity();
  TreeNodeType _kd_tree;
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  PointPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;

public:
  ICP_(const ContainerType& fixed_,
         const ContainerType& moving_,
         int min_points_in_leaf):
    _fixed(fixed_),
    _moving(moving_),
    _kd_tree(_fixed.begin(), _fixed.end(), min_points_in_leaf)
  {}

  void computeCorrespondences(){
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

  
  virtual void optimizeCorrespondences() = 0;
  
  void run(int max_iterations) {
    int current_iteration=0;
    while (current_iteration<max_iterations) {
      computeCorrespondences();
      optimizeCorrespondences();
      ++current_iteration;
      std::cerr << "Iteration: " << current_iteration;
      std::cerr << " corr: " << numCorrespondences();
      std::cerr << " inl: " << numInliers();
      std::cerr << " ker: " << numKernelized();
      std::cerr << " chi: " << _chi2_sum << std::endl;
    }
  }
  const TransformType& X() const {return _X;}
  TransformType& X()  {return _X;}
  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
};
