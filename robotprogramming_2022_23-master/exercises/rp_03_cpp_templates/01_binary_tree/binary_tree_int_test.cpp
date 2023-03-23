#include "binary_tree_int.h"
#include <iostream>

int main() {

  TreeNodeInt root(0);
  root.insert(10);
  root.insert(-10);
  root.insert(5);
  root.insert(15); 
  root.insert(-5);
  root.insert(-15);
  root.print(std::cout);
}
