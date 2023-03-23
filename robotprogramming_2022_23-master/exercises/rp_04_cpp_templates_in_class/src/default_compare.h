#pragma once

template <typename T>
struct DefaultCompare_{
  static inline bool compare(const T&a, const T&b) {
    return a<b;
  }
};
