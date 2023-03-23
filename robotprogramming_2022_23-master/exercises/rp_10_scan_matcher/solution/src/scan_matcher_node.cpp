#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include "icp/eigen_icp_2d.h"
#include "ros_bridge.h"

// Define node I/O parameters
const std::string TOPIC_SCAN = "scan";
const std::string TOPIC_ODOM = "odom";

const std::string FRAME_WORLD = "map";

ros::Publisher odom_pub;

// Scan callback definition
void scan_callback(const sensor_msgs::LaserScan::ConstPtr&);

// Fixed scan definition
std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Vector2f>> scan_fixed;
// Laser in world
Eigen::Isometry2f laser_in_world = Eigen::Isometry2f::Identity();

int main(int argc, char** argv) {
  // Initialize ROS system
  ros::init(argc, argv, "scan_matcher_node");
  // Create a NodeHandle to manage the node.
  // The "~" is used to configure the NodeHandle namespace equal to
  // the node's name.
  ros::NodeHandle nh("~");

  // Subscribe to the topic "~/scan" (globally resolved as
  // "scan_matcher_node/scan").
  // The topic should contain sensor_msgs::LaserScan messages.
  // We configure scan_callback to handle the received messages
  ros::Subscriber scan_subscriber = nh.subscribe(TOPIC_SCAN, 10, scan_callback);

  // Publish on topic "~/odom" (globally resolved as "scan_matcher/odom")
  // The topic will contain nav_msgs::Odometry messages
  odom_pub = nh.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 10);

  ros::spin();
  // In cases in which we want to execute additional code inbetween ROS update
  // routines, We could have used this different format:
  //   while (ros::ok()) {
  //     // This area can be used to execute code between ROS update routines
  //     ros::spinOnce();
  //   }

  return 0;
}

// Scan callback implementation
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg_) {
  /**
   * @brief Task of this callback is to solve a frame-to-frame lidar odometry
   * problem frame-to-frame means that we are estimating the relative ego-motion
   * of the lidar using two frames with sufficient overlap. For simplicity, we
   * are using two sequential scans which provide a lot of overlap.
   */
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> scan;
  scan2eigen(msg_, scan);

  if (scan_fixed.size() == 0) {
    scan_fixed = scan;
    ROS_INFO("Storing scan as fixed.");
    return;
  }

  // Apply ICP to estimate new pose
  ICP solver(scan_fixed, scan, 4);
  solver.run(100);

  // Update current laser_in_world estimate
  laser_in_world = laser_in_world * solver.X();

  // Prepare tf and odom messages
  geometry_msgs::TransformStamped tf_msg;
  nav_msgs::Odometry odom_msg;

  isometry2transformStamped(laser_in_world, tf_msg, FRAME_WORLD,
                            msg_->header.frame_id, msg_->header.stamp);
  transformStamped2odometry(tf_msg, odom_msg);

  // Send messages
  odom_pub.publish(odom_msg);
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(tf_msg);
  ROS_INFO("Scan [points:%ld,chi:%f,pos=(%f, %f)]", scan.size(), solver.chi(),
           laser_in_world.translation().x(), laser_in_world.translation().y());

  scan_fixed = scan;
}