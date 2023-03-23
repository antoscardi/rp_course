#include "Eigen/Geometry"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_pcl/point_types.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg2_navigation_2d_ros/tf_helpers.h>

using namespace srrg2_core;
using namespace std;

ros::Publisher scan_publisher;
std::shared_ptr<tf::TransformListener> listener;
std::string input_scan_topic          = "/scan";
std::string output_scan_topic         = "/scan_rect";
std::string odom_frame_id             = "odom";

int num_beams                         = 360;
int max_scan_buffer                    = 10;
float angle_min                       = -M_PI;
float angle_max                       = M_PI;
int direction                         = -1; //-1: clockwise, +1 counterclockwise

using LaserScanPtr= std::shared_ptr<sensor_msgs::LaserScan>;

std::list<LaserScanPtr> pending_scans;

void scanCallback(const sensor_msgs::LaserScan& scan) {
  pending_scans.push_back(LaserScanPtr(new sensor_msgs::LaserScan(scan)));
}

bool handleScan() {
  if (pending_scans.empty())
    return false;

  while ((int) pending_scans.size()>max_scan_buffer) {
    pending_scans.pop_front();
  }

  auto scan=pending_scans.front();
  const sensor_msgs::LaserScan& src=*scan;
  const std::string& scan_frame_id=src.header.frame_id;
  const ros::Time t_start = src.header.stamp-ros::Duration(src.scan_time);
  const ros::Time t_end=src.header.stamp;
  const ros::Duration time_increment = ros::Duration(src.time_increment);
  Isometry2f x_start, x_end;
  if (!getTfTransform(x_start, *listener, odom_frame_id, scan_frame_id, t_start)) {
    return false;
  }
  if (!getTfTransform(x_end, *listener, odom_frame_id, scan_frame_id, t_end)) {
    return false;
  }
  Isometry2f inv_x_end=x_end.inverse();
    sensor_msgs::LaserScan dest;
  dest.ranges.resize(num_beams);
  dest.intensities.resize(num_beams);
  dest.angle_min=angle_min;
  dest.angle_max=angle_max;
  dest.angle_increment=(angle_max-angle_min)/num_beams;
  dest.range_max=src.range_max;
  dest.range_min=src.range_min;
  dest.scan_time = 0;
  dest.time_increment = 0;
  dest.header=src.header;
  for (size_t i=0; i<dest.ranges.size(); ++i)  {
    dest.ranges[i]=dest.range_max;
    dest.intensities[i]=0;
  }
  for (size_t i=0; i<src.ranges.size(); ++i) {
    float range_i=src.ranges[i];
    if (range_i >= src.range_max || range_i<=src.range_min)
      continue;
    
    float alpha_i=src.angle_min+src.angle_increment*i;
    
    ros::Time t_i;
    if (direction==1)
      t_i  = t_end + (time_increment*i);
    else
      t_i  = t_end - (time_increment*i);
    
    float intensity_i=src.intensities[i];
    Vector2f p(range_i * cos(alpha_i), range_i * sin(alpha_i));
    Isometry2f laser_pose;
    if (!getTfTransform(laser_pose, *listener, odom_frame_id, scan_frame_id, t_i)) {
      continue;
    }
    Vector2f p2=inv_x_end*(laser_pose*p);
    float r2=p2.norm();
    if (p2.norm()>src.range_max)
      continue;
    if (p2.norm()<src.range_min)
      continue;
    float alpha2=atan2(p2.y(), p2.x());
    int idx=(alpha2-angle_min)/dest.angle_increment;
    if (idx<0 || idx>=(int)dest.ranges.size())
      continue;
    if (dest.ranges[idx]>r2) {
      dest.ranges[idx]=r2;
      dest.intensities[idx]=intensity_i;
    }
  }
  pending_scans.pop_front();
  scan_publisher.publish(dest);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("input_scan_topic", input_scan_topic);
  _nh.getParam("output_scan_topic", output_scan_topic);
  _nh.getParam("max_scan_buffer", max_scan_buffer);
  _nh.getParam("num_beams", num_beams);
  _nh.getParam("angle_min", angle_min);
  _nh.getParam("angle_max", angle_max);
  _nh.getParam("direction", direction);
  ros::Subscriber scan_subscriber = nh.subscribe(input_scan_topic, 10, scanCallback);
  scan_publisher = nh.advertise<sensor_msgs::LaserScan>(output_scan_topic, 10);
  ros::Rate loop_rate(20);
  listener.reset(new tf::TransformListener);
  while (ros::ok()) {
    ros::spinOnce();
    while (handleScan());
    loop_rate.sleep();
  }
}
