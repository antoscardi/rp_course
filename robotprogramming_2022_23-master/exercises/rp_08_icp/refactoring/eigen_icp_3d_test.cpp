#include "eigen_icp_3d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;

using Vector3f = Eigen::Vector3f;
using Vector3fVector=std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
using ICP=ICP_3D_<Vector3fVector>;

int main(int argc, char** argv) {
  using ContainerType=ICP::ContainerType;
  
  if (argc<8) {
    cerr << "usage: " << environ[0] << " tx ty tz ax ay az num_points";
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

  int num_points=atoi(argv[7]);
  
  // generate 1000 random points
  ContainerType kd_points(num_points);
  for (auto& v: kd_points) {
    v=Vector3f::Random()*100;
  }

  
  Eigen::Isometry3f iso;
  iso.linear()=Rx(ax)*Ry(ay)*Rz(az);
  iso.translation()=Vector3f(tx, ty, tz);

  ContainerType transformed_points=kd_points;
  for (auto& v: kd_points){
    v=iso*v;
  }
  ICP icp(kd_points, transformed_points, 10);
  icp.run(10);
}



