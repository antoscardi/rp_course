#pragma once
#include <iostream>
#include "default_compare.h"

struct TreeNodeInt{
  int _value;
  TreeNodeInt* _left, *_right;
  TreeNodeInt(const int& value_, TreeNodeInt* left_=0, TreeNodeInt* right_=0):
    _value(value_),
    _left(left_),
    _right(right_)
  {}

  ~TreeNodeInt(){
    if (_left) delete _left;
    if (_right) delete _right;
  }

  TreeNodeInt* insert(const int& value_) {
    DefaultCompare_<int> compare;
    if (compare(value_, _value)) {
      if (! _left) {
        _left=new TreeNodeInt(value_);
        return _left;
      }
      return _left->insert(value_);
    }
    if (compare(_value, value_)) {
      if (! _right) {
        _right=new TreeNodeInt(value_);
        return _right;
      }
      return _right->insert(value_);
    }
    return 0;
  }

  void print(std::ostream& os) {
    if (_left)
      _left->print(os);
    os << _value << " ";
    if (_right)
      _right->print(os);
  }
};
