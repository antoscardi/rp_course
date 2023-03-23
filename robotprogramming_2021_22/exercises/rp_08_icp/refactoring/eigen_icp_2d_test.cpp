#include <iostream>
#include <Eigen/StdVector>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "rotations.h"

using namespace std;
extern const char ** environ;

using Vector2f = Eigen::Vector2f;
using Vector2fVector=std::vector<Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;
using ICP=ICP_2D_<Vector2fVector>;

int main(int argc, char** argv) {
  using ContainerType=Vector2fVector;
  
  if (argc<5) {
    cerr << "usage: " << environ[0] << " tx ty az num_points";
    return -1;
  }
  float tx, ty, az;
  tx=atof(argv[1]);
  ty=atof(argv[2]);
  az=atof(argv[3]);
  cerr << "translation: " << tx << " " << ty << endl;
  cerr << "rotation: "    << az << endl;

  int num_points=atoi(argv[4]);
  
  // generate 1000 random points
  ContainerType kd_points(num_points);
  for (auto& v: kd_points) {
    v=Vector2f::Random()*100;
  }

  
  Eigen::Isometry2f iso;
  iso.linear()=Rtheta(az);
  iso.translation()=Vector2f(tx, ty);

  ContainerType transformed_points=kd_points;
  for (auto& v: kd_points){
    v=iso*v;
  }
  ICP icp(kd_points, transformed_points, 10);
  icp.run(10);
}



