#include "ros_bridge.h"

void scan2eigen(const sensor_msgs::LaserScan::ConstPtr& msg_,
                std::vector<Eigen::Vector2f,
                            Eigen::aligned_allocator<Eigen::Vector2f>>& dest_) {
  dest_.clear();
  float a = msg_->angle_min;
  float ainc = msg_->angle_increment;
  float rmin = msg_->range_min;
  float rmax = msg_->range_max;

  for (const auto& r : msg_->ranges) {
    if (r < rmin || r > rmax) continue;
    Eigen::Vector2f point(r * cos(a), r * sin(a));
    dest_.push_back(point);
    a += ainc;
  }
}

void isometry2transformStamped(const Eigen::Isometry2f& pose_,
                               geometry_msgs::TransformStamped& msg_,
                               const std::string& frame_id_,
                               const std::string& child_frame_id_,
                               const ros::Time& stamp_) {
  msg_.header.frame_id = frame_id_;
  msg_.child_frame_id = child_frame_id_;
  msg_.header.stamp = stamp_;
  msg_.transform.translation.x = pose_.translation().x();
  msg_.transform.translation.y = pose_.translation().y();
  msg_.transform.translation.z = 0;

  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  R.block<2, 2>(0, 0) = pose_.linear();
  Eigen::Quaternionf q(R);
  q.normalize();
  msg_.transform.rotation.w = q.w();
  msg_.transform.rotation.x = q.x();
  msg_.transform.rotation.y = q.y();
  msg_.transform.rotation.z = q.z();
}

void transformStamped2odometry(const geometry_msgs::TransformStamped& msg_,
                               nav_msgs::Odometry& odom_) {
  odom_.header.frame_id = msg_.header.frame_id;
  odom_.header.stamp = msg_.header.stamp;
  odom_.child_frame_id = msg_.child_frame_id;
  odom_.pose.pose.position.x = msg_.transform.translation.x;
  odom_.pose.pose.position.y = msg_.transform.translation.y;
  odom_.pose.pose.position.z = msg_.transform.translation.z;

  odom_.pose.pose.orientation.w = msg_.transform.rotation.w;
  odom_.pose.pose.orientation.x = msg_.transform.rotation.x;
  odom_.pose.pose.orientation.y = msg_.transform.rotation.y;
  odom_.pose.pose.orientation.z = msg_.transform.rotation.z;
}
