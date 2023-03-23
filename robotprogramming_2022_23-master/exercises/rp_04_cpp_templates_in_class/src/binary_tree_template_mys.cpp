#include "binary_tree_template.h"
#include <iostream>
#include <string>

using namespace std;

struct MyS{
  MyS(const float& f_=0,
         const double& d_=0,
         const string& s_=string("")):
    f(f_),
    d(d_),
    s(s_){}
  
  float f;
  double d;
  string s;

  bool operator < (const MyS& b) const {
      return (f<b.f)
        || (f==b.f && d<b.d)
        || (f==b.f && d==b.d && s<b.s);
  }

};

ostream& operator << (ostream& os, const MyS& item) {
  os << "addr: " << &item
     << " f: " << item.f
     << " d: " << item.d 
     << " s: " << item.s << endl;
  return os;

}

struct MySCompare {
  inline static bool compare(const MyS&a, const MyS&b) {
    return
      (a.f<b.f)
      || (a.f==b.f && a.d<b.d)
      || (a.f==b.f && a.d==b.d && a.s<b.s);
  }
};

using TreeNodeMyS=TreeNode_<MyS>;

int main() {

  TreeNodeMyS root(MyS(0.7, 0.1, "boh"));
  root.insert(MyS(0.2, 0.1, "boh"));
  root.insert(MyS(0.7, 0.1, "bah"));
  root.insert(MyS(0.8, 0.3, "boh"));
  root.insert(MyS(0.5, 0.1, "boh"));
  root.insert(MyS(0.5, 0.8, "boh"));
  root.print(std::cout);
}
