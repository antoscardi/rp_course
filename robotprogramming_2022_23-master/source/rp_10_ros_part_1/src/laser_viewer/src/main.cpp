#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <Eigen/Core>

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  using namespace std;
  cout << "set size ratio -1" <<endl;
  cout << "plot '-' w p" <<endl;
  float alpha=msg->angle_min;
  for (const float&r : msg->ranges) {
    if (r > msg->range_min && r<msg->range_max) {
      Eigen::Vector2f p(r*cos(alpha),r*sin(alpha));
      cout << p.transpose() << endl;
    }
    alpha += msg->angle_increment;
  }
  cout << "e" << endl;
}

int main(int argc, char** argv){
  std::cerr << "j l'avemo fatta" << std::endl;
  ros::init(argc, argv, "laser_viewer");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/base_scan", 1000, laserCallback);
  ros::spin();
}
