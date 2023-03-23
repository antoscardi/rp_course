#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv) {
  if (argc<2)
    return -1;
  // we open a file stream;

  std::ifstream is(argv[1]);
  Vector3fVector points;
  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;
  savePoints(std::cerr, points);


  /* TODO for you 

     1. change the above program to read n points in 2D
     2. change the program to read n points in 2D and 3D from a std::list

   */
  return 0;
}

