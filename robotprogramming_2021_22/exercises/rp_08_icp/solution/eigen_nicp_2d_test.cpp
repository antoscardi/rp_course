#include "eigen_nicp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include <iostream>
#include <fstream>

using namespace std;
extern const char ** environ;


using PointNormal2fVector=std::vector<PointNormal2f, Eigen::aligned_allocator<PointNormal2f> >;

void drawLine(PointNormal2fVector& dest,
              const Vector2f& p0,
              const Vector2f& p1,
              float density=10) {
  Vector2f delta=p1-p0;
  float length=delta.norm();
  int n_points=length*density;
  Vector2f normal(delta.y(), -delta.x());
  normal.normalize();
  Vector2f d=delta/n_points;
  for (int i=0; i<n_points; ++i) {
    PointNormal2f p;
    p.head<2>()=p0+d*float(i);
    p.tail<2>()=normal;
    dest.push_back(p);
  }
}
int main(int argc, char** argv) {
  using ContainerType=NICP::ContainerType;
  
  if (argc<5) {
    cerr << "usage: " << environ[0] << " tx ty az num_lines | gnuplot" << endl;
    cerr << "example: 0.3 0.3 0.5 4 | gnuplot" << endl;
    cerr << "focus on terminal window and press a key + enter tp progress iterations" << endl;
    return -1;
  }
  float tx, ty, az;
  tx=atof(argv[1]);
  ty=atof(argv[2]);
  az=atof(argv[3]);
  cerr << "translation: " << tx << " " << ty << endl;
  cerr << "rotation: "    << az << endl;

  int num_lines=atoi(argv[4]);
  
  // generate N virtuual lines on points, with consistent normals
  ContainerType kd_points;
  for (int i=0; i<num_lines; ++i) {
    Vector2f p0=(Vector2f::Random()-Vector2f(-0.5, -0.5))*5;
    Vector2f p1=(Vector2f::Random()-Vector2f(-0.5, -0.5))*5;
    drawLine(kd_points, p0, p1);
  }

  
  Eigen::Isometry2f iso;
  iso.linear()=Rtheta(az);
  iso.translation()=Vector2f(tx, ty);

  ContainerType transformed_points=kd_points;
  for (auto& v: kd_points){
    v=iso*v;
  }
  NICP icp(kd_points, transformed_points, 20);
  icp.run(100);
}



