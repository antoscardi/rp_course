#include <iostream>
#include <cmath>
#include "simple_geometry.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include <sys/time.h>

using namespace std;

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv,0);
  return tv.tv_sec*1000+tv.tv_usec*1e-3;
}

int main(int argc, char** argv) {
  World w;
  w.loadFromImage(argv[1]);

  IntPoint middle(w.rows/2, w.cols/2);
  Pose robot_pose;
  robot_pose.translation = w.grid2world(middle);
  Robot r(0.3, &w, robot_pose);
  Lidar l(M_PI,10,180,&r, Pose(0.1,0,0));

  Robot r2(0.3, &w, Pose(1,10,0)*robot_pose);
  Lidar l2(M_PI,10,180,&r2, Pose(0.1,0,0));

  Robot r3(0.3, &w, Pose(1,-10,0)*robot_pose);
  Lidar l3(M_PI,10,180,&r3, Pose(0.1,0,0));
  
  float delay = 0.1;
  int k;
  while (1) {
    double t_start=timeMillisec();
    w.timeTick(delay);
    double t_end=timeMillisec();
    cerr << "duration" << t_end-t_start << endl;
    cerr << "image_size: " << w.rows << " " << w.cols << endl;
    w.draw();
    
    //k=cv::waitKeyEx(delay*1000)&255;
    k=cv::waitKeyEx(0)&255;
    switch (k) {
    case 81: r.rv+=0.05; break;// arow left
    case 82: r.tv+=0.1; break;// arow up
    case 83: r.rv-=0.05; break;// arow right
    case 84: r.tv-=0.1; break;// arow dw
    case 32: r.tv=0; r.rv=0; break;// spacebar
    case 27: return 0; //space
    default:;
    }
    cerr << "k: " << (int) k << endl;
  }
}


// to be blasted
/*
 // Point uno (1, 1);
  // Point due (2, -2);
  // cerr << uno-due << endl;
  // cerr << uno+due << endl;
  // cerr << due * 0.5 << endl;

  // Pose t1(1, 1, 0);
  // Pose t2(1, 0, M_PI/2);
  // cerr << t1 << endl;
  // cerr << t2 << endl;
  // cerr << t1*t2 << endl;
  // cerr << (t1*t2)*(uno) << endl;

  Point p1 (20, 10);
  IntPoint ip1 = w.world2grid(p1);
  Point p2 = w.grid2world(ip1);

  cerr << "p1 " << p1 <<  endl;
  cerr << "ip1 " << ip1 <<  endl;
  cerr << "p2 " << p2 <<  endl;

  int num_collisions=0;

  IntPoint center(w.rows/2, w.cols/2);
  for (int i=0; i<50; ++i) {
    cerr << "radius: " << i << " ";
    if (w.collides(center, i)) {
      cerr << " collision" << endl;
    } else {
      cerr << " free" << endl;
    }
  }
  return 0;
  
  for (int r = 0; r<w.rows; ++r) {
    for (int c = 0; c<w.cols; ++c){
      if (w.collides(IntPoint(r,c), 10)) {
        //cerr << "me collide " << r << " " << c << endl;
        ++num_collisions;
      }
      
    }
  }
  cerr << "coll :" << num_collisions << " "  << w.size << endl;
*/
