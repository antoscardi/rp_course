#include "Eigen/Geometry"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_pcl/point_types.h"
#include "tf/transform_listener.h"
#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/Twist.h>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg2_navigation_2d_ros/tf_helpers.h>

using namespace srrg2_core;
std::shared_ptr<tf::TransformListener> listener;

std::string map_frame_id       = "map";
std::string base_link_frame_id = "base_link";
std::string laser_topic        = "/base_scan";
std::string odom_topic         = "/camera/odom/sample";
double time_lag                = 1.;
std::map<double, sensor_msgs::LaserScan> laser_msgs;
std::map<double, nav_msgs::Odometry> odom_msgs;

double last_laser_update_time = 0;
double last_odom_update_time  = 0;

void laserCallback(const sensor_msgs::LaserScan& scan) {
  laser_msgs.insert(std::make_pair(scan.header.stamp.toSec(), scan));
  last_laser_update_time = std::max(last_laser_update_time, scan.header.stamp.toSec());
}

void odomCallback(const nav_msgs::Odometry& odom) {
  odom_msgs.insert(std::make_pair(odom.header.stamp.toSec(), odom));
  last_odom_update_time = std::max(last_odom_update_time, odom.header.stamp.toSec());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("map_frame_id", map_frame_id);
  _nh.getParam("base_link_frame_id", base_link_frame_id);
  _nh.getParam("laser_topic", laser_topic);
  _nh.getParam("time_lag", time_lag);
  std::cerr << argv[0] << ": running with these parameters" << std::endl;
  std::cerr << "_map_frame_id:=" << map_frame_id << std::endl;
  std::cerr << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
  std::cerr << "_laser_topic:=" << laser_topic << std::endl;
  std::cerr << "_odom_topic:=" << odom_topic << std::endl;

  ros::Subscriber odom_subscriber  = nh.subscribe(odom_topic, 10, odomCallback);
  ros::Subscriber laser_subscriber = nh.subscribe(laser_topic, 10, laserCallback);
  ros::Rate loop_rate(50);
  ros::Duration tfCacheDuration;
  tfCacheDuration = tfCacheDuration.fromSec(600);
  listener.reset(new tf::TransformListener(tfCacheDuration));

  std::ofstream laser_stream("laser_dumped.txt");
  Eigen::Isometry2f laser_pose;
  laser_stream << std::setprecision(9);
  laser_stream << std::fixed;

  std::ofstream odom_stream("odom_dumped.txt");
  odom_stream << std::setprecision(9);
  odom_stream << std::fixed;

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    while (!laser_msgs.empty()) {
      const sensor_msgs::LaserScan& scan = laser_msgs.begin()->second;
      if (last_laser_update_time - scan.header.stamp.toSec() < time_lag) {
        std::cerr << "l";
        break;
      }
      if (listener->canTransform(map_frame_id, scan.header.frame_id, scan.header.stamp)) {
        Eigen::Isometry2f laser_pose;
        getTfTransform(
          laser_pose, *listener, map_frame_id, scan.header.frame_id, scan.header.stamp);
        std::cerr << "L";
        laser_stream << scan.header.stamp.toSec() << std::endl;
      }
      laser_msgs.erase(laser_msgs.begin());
    }

    while (!odom_msgs.empty()) {
      const nav_msgs::Odometry& odom = odom_msgs.begin()->second;
      Eigen::Isometry2f odom_pose;
      if (last_odom_update_time - odom.header.stamp.toSec() < time_lag) {
        std::cerr << "o";
        break;
      }
      if (listener->canTransform(map_frame_id, base_link_frame_id, odom.header.stamp)) {
        getTfTransform(odom_pose, *listener, map_frame_id, base_link_frame_id, odom.header.stamp);
        std::cerr << "O";
        odom_stream << odom.header.stamp.toSec() << std::endl;
      }
      odom_msgs.erase(odom_msgs.begin());
    }
  }
}
