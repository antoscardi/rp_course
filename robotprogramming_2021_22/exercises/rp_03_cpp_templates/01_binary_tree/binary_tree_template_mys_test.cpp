#include "binary_tree_template.h"
#include <iostream>

struct MyS {
  int i; float f;
  MyS(int i_, float f_): i(i_),f(f_){};
};

std::ostream& operator<< (std::ostream& os, const MyS& s) {
  os << "[" << s.i << "," << s.f << "]";
};

struct MySCompare {
  bool operator()(const MyS&a, const MyS& b) const {
    return a.i < b.i;
  }
};

  
using TreeNodeMyS=TreeNode_<MyS, MySCompare>;
int main() {

  TreeNodeMyS root(MyS(0, 0.1));
  root.insert(MyS(10, 0.2));
  root.insert(MyS(-10, 0.2));
  root.insert(MyS(-5, 0.3));
  root.insert(MyS(-5, 0.4));
  root.insert(MyS(15, 0.3));
  root.insert(MyS(-15, 0.4));
  root.print(std::cout);
}
