#include "eigen_01_point_loading.h"
#include "eigen_covariance.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  if (argc<2)
    return -1;

  std::ifstream is(argv[1]);
  Vector3fVector points;
  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;
  savePoints(std::cerr, points);
  
  Matrix3f cov;
  Vector3f mean;
  computeMeanAndCovariance(mean, cov, points.begin(), points.end());
  Vector3f largest=largestEigenVector(cov);
  std::cerr << "mean: " << mean.transpose() << endl;
  std::cerr << "cov: " << endl << cov << endl;
  std::cerr << "lv: " << largest.transpose() << endl;
}

