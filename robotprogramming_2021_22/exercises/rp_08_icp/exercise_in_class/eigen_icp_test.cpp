#include "eigen_icp.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;


using Vector3fVector=std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;

int main(int argc, char** argv) {
  using ContainerType=ICP::ContainerType;
  
  if (argc<8) {
    cerr << "usage: " << argv[0] << " tx ty tz ax ay az num_points";
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
  
  // generate N random points
  ContainerType fixed_points(num_points);
  for (auto& v: fixed_points) {
    v=Vector3f::Random()*100;
  }

  
  Eigen::Isometry3f iso;
  iso.linear()=Rx(ax)*Ry(ay)*Rz(az);
  iso.translation()=Vector3f(tx, ty, tz);

  cerr << "GUESS transform" << endl;
  cerr << iso.matrix() << endl;

  
  ContainerType moving_points=fixed_points;
  for (auto& v: moving_points){
    v=iso.inverse()*v;
  }
  ICP icp(fixed_points, moving_points, 10);
  icp.run(10);
}



