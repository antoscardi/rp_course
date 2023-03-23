#include <iostream>
using namespace std;

template <typename T>
const T& myMinTemplate(const T&a, const T&b) {
  if (a<b) return a;
  return b;
}

// recursion example (nasty)
template <int i>
inline int factorial_() {
  return i*factorial_<i-1>();
}

// partial specialization
template<>
inline int factorial_<0>() {
  return 1;
}

int main(){
  cout << myMinTemplate(3,4) << endl;
  cout << myMinTemplate(4.5,3.8) << endl;
  
  cout << factorial_<5>() << std::endl;
}
