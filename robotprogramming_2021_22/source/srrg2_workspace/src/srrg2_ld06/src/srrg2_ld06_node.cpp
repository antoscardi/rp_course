#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include "ld_06.h"
using namespace std;

static sensor_msgs::LaserScan pub_scan;

ros::Publisher pub;
int publishScan(Scan* scan) {
  if (! ros::ok())
    return 0;
  pub_scan.header.stamp=ros::Time(scan->timestamp);
  pub_scan.angle_min=scan->angle_min;
  pub_scan.angle_max=scan->angle_max;
  pub_scan.angle_increment=scan->angle_increment;
  pub_scan.scan_time=scan->scan_time;
  pub_scan.time_increment=scan->scan_time/scan->num_beams;
  pub_scan.range_min=0.1;
  pub_scan.range_max=8;
  pub_scan.ranges.resize(scan->num_beams);
  pub_scan.intensities.resize(scan->num_beams);
  int k=scan->num_beams-1;
  for (int i=0; i<scan->num_beams; ++i, --k) {
    pub_scan.ranges[k]=scan->ranges[i];
    pub_scan.intensities[k]=scan->intensities[i];
  }
  pub.publish(pub_scan);
  ros::spinOnce();
  return 1;
}

int main(int argc, char** argv) {
  std::string topic = "/base_scan";
  std::string frame_id = "/laser_frame";
  std::string serial_device = "/dev/ttyUSB0";
  ros::init(argc, argv, "ld06",ros::init_options::AnonymousName);
  ros::NodeHandle n("~");

  cerr << "running with params: " << endl;
  cerr << "_serial_device: " << serial_device << endl;
  cerr << "_frame_id: " << frame_id << endl;
  cerr << "_topic: " << topic << endl;
  
  pub = n.advertise<sensor_msgs::LaserScan>(topic, 10);
  pub_scan.header.frame_id=frame_id;
  runLaser(publishScan, 360, serial_device.c_str());
  return 0;
  
}
