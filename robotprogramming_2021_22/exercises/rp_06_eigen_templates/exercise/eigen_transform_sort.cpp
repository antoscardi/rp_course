#include <Eigen/Geometry>
#include "eigen_01_point_loading.h"
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;
using namespace Eigen;
extern const char**environ;

inline Eigen::Matrix3f Rx(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    1,  0,  0,
    0,  c, -s,
    0,  s,  c;
  return R;
}


inline Eigen::Matrix3f Ry(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c,  0,  s,
    0,  1,  0,
   -s,  0,  c;
  return R;
}

inline Eigen::Matrix3f Rz(float theta) {
  float c=cos(theta);
  float s=sin(theta);
  Eigen::Matrix3f R;
  R <<
    c, -s, 0,
    s,  c, 0,
    0,  0, 1;
  return R;
}


template <int idx>
struct CoordinateCompare_{
  // TODO
};

int main(int argc, char** argv) {
  if (argc<8) {
    cerr << "usage: " << environ[0] << " tx ty tz ax ay az filename";
    return -1;
  }
  float ax, ay, az, tx, ty, tz;
  tx=atof(argv[1]);
  ty=atof(argv[2]);
  tz=atof(argv[3]);
  ax=atof(argv[4]);
  ay=atof(argv[5]);
  az=atof(argv[6]);
  cerr << "translation: " << tx << " " << ty << " " << tz << endl;
  cerr << "rotation: "    << ax << " " << ay << " " << az << endl;

  std::ifstream is(argv[7]);

  Vector3fVector points;
  int num_points = loadPoints(points, is);
  cerr << "I read" << num_points << "from the stream " << endl;

  Eigen::Isometry3f iso;
  iso.linear()=Rx(ax)*Ry(ay)*Rz(az);
  iso.translation()=Vector3f(tx, ty, tz);

  cerr << "Isometry: " << endl;
  cerr <<  iso.matrix() << endl;
  
  Vector3fVector transformed_points(points.size());
  for (size_t i=0; i<points.size(); ++i)
    transformed_points[i] = iso*points[i];

  bool run=true;
  while (1) {
    cerr << "Sorting: x, y, z" << endl;
    char s;
    cin >> s;
    switch (s){
    case 'x':
      cerr << "sorting by x" << endl;
      std::sort(transformed_points.begin(), transformed_points.end(), CoordinateCompare_<0>());
      break;
    case 'y':
      cerr << "sorting by y" << endl;
      sorter.idx=1;
      std::sort(transformed_points.begin(), transformed_points.end(), CoordinateCompare_<1>());
      break;
    case 'z':
      cerr << "sorting by z" << endl;
      sorter.idx=2;
      std::sort(transformed_points.begin(), transformed_points.end(), CoordinateCompare_<2>());
      break;
    default:
      cout << "invalid option, exiting" << endl;
      run=false;
    }
    if (! run)
      break;
    for(const auto& p: transformed_points){
      cerr << p.transpose() << endl;
    }
  }

  return 0;
}

