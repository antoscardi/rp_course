#pragma once
#include "default_compare.h"
#include <iostream>

template <typename T_,
          typename Compare_=DefaultCompare_<T_> >

struct TreeNode_{
  using ValueType=T_;
  using ThisType=TreeNode_<ValueType, Compare_>;
  static const Compare_ _compare;
  ValueType _value;
  ThisType* _left, *_right;

  TreeNode_(const ValueType& value_,
            ThisType* left_=0,
            ThisType* right_=0):
    _value(value_),
    _left(left_),
    _right(right_)
  {}

  ~TreeNode_(){
    if (_left) delete _left;
    if (_right) delete _right;
  }

  ThisType* insert(const ValueType& value_) {
    if (_compare(value_, _value)) {
      if (! _left) {
        _left=new ThisType(value_);
        return _left;
      }
      return _left->insert(value_);
    }
    if (_compare(_value, value_)) {
      if (! _right) {
        _right=new ThisType(value_);
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

// out of class static member declaration
template <typename T_, typename Compare_>
const Compare_ TreeNode_<T_,Compare_>::_compare;
//    ^^^^^^^^ ^^^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^
//    type     class                   member
