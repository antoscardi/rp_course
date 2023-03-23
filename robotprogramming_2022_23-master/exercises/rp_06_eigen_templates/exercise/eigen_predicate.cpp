#include <fstream>
#include <iostream>

#include "eigen_01_point_loading.h"

using namespace std;
using namespace std;

using namespace std;
using namespace std;

template <typename VectorType_>
struct PlaneSidePredicate_ {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using VectorType = VectorType_;
  VectorType _m;
  VectorType _n;
  PlaneSidePredicate_(const VectorType& m_, const VectorType& n_)
      : _m(m_), _n(n_) {}

  inline bool operator()(const VectorType& p) const {
    // get from the VectorType the type of its field;
    using ScalarType = typename VectorType::Scalar;
    // Todo: what should I return that is true if less than 0 ?
    return true;
  }
};

template <typename IteratorType_, typename PredicateType_>
IteratorType_ split(IteratorType_ begin, IteratorType_ end,
                    PredicateType_ predicate) {
  using ValueType = typename IteratorType_::value_type;
  auto lower = begin;
  auto upper = std::make_reverse_iterator(end);
  while (lower != upper.base()) {
    ValueType& v_lower = *lower;
    ValueType& v_upper = *upper;
    if (predicate(v_lower)) {
      ++lower;
    } else {
      std::swap(v_lower, v_upper);
      ++upper;
    }
  }
  return upper.base();
}

int main(int argc, char** argv) {
  if (argc < 2) return -1;

  std::ifstream is(argv[1]);
  Vector3fVector points;
  Vector3f m;
  m.setZero();
  Vector3f n;
  n << 1, 0, 0;  // split along z

  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;

  split(points.begin(), points.end(), PlaneSidePredicate_<Vector3f>(m, n));
  savePoints(cerr, points);

  return 0;
}
