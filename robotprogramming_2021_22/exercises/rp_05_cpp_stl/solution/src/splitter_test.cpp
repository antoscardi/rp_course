#include "point_with_index.h"
#include <list>
#include <fstream>
#include <algorithm>
#include <cassert>

using namespace rp;
using namespace std;


// split functions
// takes two iterators (begin and end)
// and reorders the data so that the items for which the predicate
// is true are at the beginning, tho others at the end
// it returns the iterator to the splitting element between the two classes
template <typename IteratorType_, typename PredicateType_>
IteratorType_ split(IteratorType_ begin,
                    IteratorType_ end,
                    PredicateType_ predicate) {
  using ValueType    = typename IteratorType_::value_type;
  auto lower=begin;  //first
  // upper is an iterator at the end, it is a reverse iterator,
  // and moves backward when incremented;
  // we apply the predicate to each point,
  // and if the result is positive
  //   we leave the point where it is
  // otherwise
  //   we move the point in the other extrema, and increment the extrema
  // the iterations end when the upper and lower ends match
  auto upper=std::make_reverse_iterator(end); 

  // upper.base returns the normal iterator out of the reverse one
  while (lower!=upper.base()) {
    ValueType& v_lower=*lower;
    ValueType& v_upper=*upper;
    if ( predicate(v_lower) ){
      ++lower;
    } else {
      std::swap(v_lower,v_upper);
      ++upper;
    }
  }
  return upper.base();
}


using PointWithIndexList=std::list<PointWithIndex>;

struct OddFirst{
  bool operator()(const PointWithIndex& p){
    return p.index%2;
  }
};

int main(int argc, char** argv) {
  assert(argc>1);
  ifstream is(argv[1]);
  PointWithIndexList points;
  while (is) {
    PointWithIndex p;
    is >> p;
    if (is) {
      points.push_back(p);
    }
  }
  cerr << "loaded " << points.size()
       << " points from file [" << argv[1] << "]" << endl;
  split(points.begin(), points.end(), OddFirst());

  for(const auto& p: points)
    cerr << p << endl;
  
}
