#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace std;

using IntVector=std::vector<int>;

int main(int argc, char** argv) {
  assert(argc>1);
  ifstream is(argv[1]);
  IntVector values;
  while (is) {
    int i;
    is >> i;
    if (is) {
      values.push_back(i);
    }
  }
  std::sort(values.begin(), values.end());
  for (auto& v: values)
    cout << v << endl;
}
