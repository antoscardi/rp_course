#pragma once
#include <iostream>
#include "default_compare.h"


struct TreeNodeInt{
  int _value;
  TreeNodeInt* _left, *_right;

  static const DefaultCompare_<int> compare;
  
  TreeNodeInt(const int& value_,
              TreeNodeInt* left_=0,
              TreeNodeInt* right_=0):
    _value(value_),
    _left(left_),
    _right(right_)
  {}

  ~TreeNodeInt(){
    if (_left) delete _left;
    if (_right) delete _right;
  }

  TreeNodeInt* insert(const int& new_value) {
    if (compare(new_value, _value)) {
      if (! _left) {
        _left=new TreeNodeInt(new_value);
        return _left;
      }
      return _left->insert(new_value);
    }
    if (compare(_value, new_value)) {
      if (! _right) {
        _right=new TreeNodeInt(new_value);
        return _right;
      }
      return _right->insert(new_value);
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

// binary_tree_int.cpp
const DefaultCompare_<int>  TreeNodeInt::compare;
