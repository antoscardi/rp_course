#pragma once
#include "planner_2d.h"

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  template <typename IteratorType_>
  struct DefaultAccessor_ {
    inline Vector2i operator()(IteratorType_& it) {
      return *it;
    }
  };

  template <typename IteratorType_>
  struct CoordinatesAccessor_ {
    inline Vector2i operator()(IteratorType_& it) {
      return it->coordinates();
    }
  };

  template <typename IteratorType_, typename AccessorType_ = DefaultAccessor_<IteratorType_>>
  void paintPoints(cv::Mat& dest_, IteratorType_ begin_, IteratorType_ end_) {
    AccessorType_ acc;
    int rows = dest_.rows;
    int cols = dest_.cols;
    for (IteratorType_ it = begin_; it != end_; ++it) {
      Vector2i pt = acc(it);
      if (pt.x() >= rows || pt.y() >= cols || pt.x() < 0 || pt.y() < 0)
        continue;
      auto& color = dest_.at<uint8_t>(cv::Point2i(pt.y(), pt.x()));
      color       = 255;
    }
  }

}
