#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "eigen_01_point_loading.h"

template <typename Iterator_>
int computeMeanAndCovariance(
    Eigen::Matrix<typename Iterator_::value_type::Scalar,
                  Iterator_::value_type::RowsAtCompileTime, 1>& mean,

    Eigen::Matrix<typename Iterator_::value_type::Scalar,
                  Iterator_::value_type::RowsAtCompileTime,
                  Iterator_::value_type::RowsAtCompileTime>& cov,
    Iterator_ begin, Iterator_ end) {
  using PointType = typename Iterator_::value_type;
  using Scalar = typename PointType::Scalar;
  constexpr int Dim = PointType::RowsAtCompileTime;
  using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;

  mean.setZero();
  cov.setZero();

  Iterator_ start = begin;
  int count = 0;
  while (start != end) {
    // Todo: Accumulate points on mean
    ++start;
    ++count;
  }
  mean /= Scalar(count);

  start = begin;
  while (start != end) {
    // Todo: Accumulate the covariance components (look at the slides)
    ++start;
  }
  cov /= Scalar(count - 1);

  return 0;
}

// this invokes the Eigen functions to compute the eigenvalues of
// a symmetric matrix. We extract the largest EigenVector of the covariance
// which denotes the direction of highest variation.
template <typename SquareMatrixType_>
Eigen::Matrix<typename SquareMatrixType_::Scalar,
              SquareMatrixType_::RowsAtCompileTime, 1>
largestEigenVector(const SquareMatrixType_& m) {
  Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
  es.compute(m);
  return es.eigenvectors().col(SquareMatrixType_::RowsAtCompileTime - 1);
}
