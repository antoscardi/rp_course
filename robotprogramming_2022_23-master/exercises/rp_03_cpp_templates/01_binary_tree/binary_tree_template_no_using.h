#pragma once
#include "default_compare.h"
#include <iostream>

template <typename T_, typename Compare_=DefaultCompare_<T_> >
struct TreeNode_{
  T_ _value;
  TreeNode_<T_,Compare_>* _left, *_right;
  TreeNode_(const T_& value_, TreeNode_<T_,Compare_>* left_=0, TreeNode_<T_,Compare_>* right_=0):
    _value(value_),
    _left(left_),
    _right(right_)
  {}

  ~TreeNode_(){
    if (_left) delete _left;
    if (_right) delete _right;
  }

  TreeNode_<T_,Compare_>* insert(const T_& value_) {
    Compare_ _compare;
    if (_compare(value_, _value)) {
      if (! _left) {
        _left=new TreeNode_<T_,Compare_>(value_);
        return _left;
      }
      return _left->insert(value_);
    }
    if (_compare(_value, value_)) {
      if (! _right) {
        _right=new TreeNode_<T_,Compare_>(value_);
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
