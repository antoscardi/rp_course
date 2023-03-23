#include "Eigen/Geometry"
#include "srrg_data_structures/path_matrix_distance_search.h"
#include "srrg_image/image.h"
#include "srrg_pcl/point_types.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_navigation_2d_msgs/CollisionAvoiderStatus.h>
#include <srrg2_navigation_2d_ros/tf_helpers.h>

using namespace srrg2_core;

// working globals
ros::Publisher cmd_vel_publisher;
ros::Publisher status_publisher;
std::shared_ptr<tf::TransformListener> listener;
Point2fVectorCloud last_scan_points;
double last_scan_stamp = 0;

// hardcoded parameters
float action_force_range         = 0.5;
std::string cmd_vel_input_topic  = "/cmd_vel_input";
std::string cmd_vel_output_topic = "/cmd_vel";
std::string scan_topic           = "/base_scan";
std::string status_topic         = "/collision_avoider_status";
std::string base_link_frame_id   = "/base_link";
float robot_radius               = 0.2;
float voxelize_res               = 0.05;
float angular_loss               = 10;
float max_angular_correction     = 0.5;
bool verbose                     = false;
float linear_velocity_min        = 1e-2;

srrg2_navigation_2d_msgs::CollisionAvoiderStatus status_msg;

bool scan2points(Point2fVectorCloud& dest, const sensor_msgs::LaserScan& scan) {
  Isometry2f laser_pose;
  std::cerr << "scan measurements: " << scan.ranges.size() << std::endl;
  if (!getTfTransform(
        laser_pose, *listener, base_link_frame_id, scan.header.frame_id, ros::Time(0)))
    return false;
  // float crop_range = scan.range_max;
  float crop_range = std::min(action_force_range, scan.range_max);
  Point2fVectorCloud scan_points_all;
  scan_points_all.reserve(scan.ranges.size());
  std::cout << "size scan_points_all: " << scan_points_all.size() << std::endl;
  scan_points_all.clear();
  float alpha = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i, alpha += scan.angle_increment) {
    std::cerr << "In for loop!" << std::endl;
    float r = scan.ranges[i];
    if (r < scan.range_min) {
      std::cerr << "first if!!!!" << std::endl;
      continue;
    }

    if (r > crop_range) {
      std::cerr << "second if!!!!" << std::endl;
      continue;
    }

    float c = cos(alpha), s = sin(alpha);
    Point2f p;
    p.coordinates().x() = r * c;
    p.coordinates().y() = r * s;
    std::cout << "p: " << p.coordinates().x() << std::endl;
    std::cout << "p: " << p.coordinates().y() << std::endl;
    std::cerr << "end for loop!" << std::endl;
    scan_points_all.emplace_back(p);
  }
  std::cout << "size: " << scan_points_all.size() << std::endl;
  scan_points_all.transformInPlace(laser_pose);
  std::cout << "size: " << scan_points_all.size() << std::endl;

  Vector2f voxelize_coeffs(voxelize_res, voxelize_res);
  last_scan_points.clear();
  scan_points_all.voxelize(std::back_insert_iterator<Point2fVectorCloud>(last_scan_points),
                           voxelize_coeffs);
  return true;
}

void cmdVelCallback(const geometry_msgs::Twist& twist_input) {
  status_msg.header.stamp  = ros::Time::now();
  status_msg.cmd_vel_input = twist_input;
  if (!last_scan_points.size() || twist_input.linear.x < 0) {
    cmd_vel_publisher.publish(twist_input);
    status_msg.cmd_vel_output = twist_input;
    status_msg.status         = "clear";
    status_publisher.publish(status_msg);
    return;
  }
  // compute the twist variation

  float linear_force  = 0;
  float angular_force = 0;
  const Vector2f tv_input(twist_input.linear.x, twist_input.linear.y);
  float tv_magnitude = tv_input.norm();
  float max_f        = 0;
  for (size_t i = 0; i < last_scan_points.size(); ++i) {
    const Vector2f& p         = last_scan_points[i].coordinates();
    float distance_to_contour = std::max(0.f, float(p.norm() - robot_radius));
    float magnitude           = tv_magnitude * (1. / (0.5f + distance_to_contour));
    Vector2f f                = p.normalized() * magnitude;
    float df                  = f.dot(tv_input);
    max_f                     = std::max(max_f, df);
    linear_force -= df;
    angular_force += f.x() * tv_input.y() - f.y() * tv_input.x();
  }
  if (linear_force)
    angular_force /= angular_loss * fabs(linear_force);

  geometry_msgs::Twist twist_output = twist_input;
  twist_output.linear.x -= max_f;
  twist_output.linear.x = std::max(0.f, (float) twist_output.linear.x);
  if (twist_output.linear.x < linear_velocity_min) {
    status_msg.status     = "blocked";
    twist_output.linear.x = 0;
  } else {
    status_msg.status = "adjusting";
  }
  float naf = fabs(angular_force);
  if (naf > max_angular_correction)
    angular_force *= max_angular_correction / naf;
  twist_output.angular.z += angular_force;

  if (verbose) {
    std::cerr << "np: " << last_scan_points.size() << "lf: " << linear_force << " "
              << "in: [" << twist_input.linear.x << " " << twist_input.angular.z << "] "
              << "co: [" << -max_f << " " << angular_force << "] "
              << "ou: [" << twist_output.linear.x << " " << twist_output.angular.z << "] "
              << std::endl;
  }
  status_msg.cmd_vel_output = twist_output;
  cmd_vel_publisher.publish(twist_output);
  status_publisher.publish(status_msg);
}

void scanCallback(const sensor_msgs::LaserScan& scan) {
  if (!scan2points(last_scan_points, scan))
    return;
  last_scan_stamp = scan.header.stamp.toSec();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("action_force_range", action_force_range);
  _nh.getParam("cmd_vel_input_topic", cmd_vel_input_topic);
  _nh.getParam("cmd_vel_output_topic", cmd_vel_output_topic);
  _nh.getParam("scan_topic", scan_topic);
  _nh.getParam("base_link_frame_id", base_link_frame_id);
  _nh.getParam("robot_radius", robot_radius);
  _nh.getParam("voxelize_res", voxelize_res);
  _nh.getParam("angular_loss", angular_loss);
  _nh.getParam("max_angular_correction", max_angular_correction);
  _nh.getParam("verbose", verbose);
  std::cerr << argv[0] << ": running with params" << std::endl;

  std::cerr << "_action_force_range:=" << action_force_range << std::endl;
  std::cerr << "_cmd_vel_input_topic:=" << cmd_vel_input_topic << std::endl;
  std::cerr << "_cmd_vel_output_topic:=" << cmd_vel_output_topic << std::endl;
  std::cerr << "_scan_topic:=" << scan_topic << std::endl;
  std::cerr << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
  std::cerr << "_status_topic:=" << status_topic << std::endl;
  std::cerr << "_robot_radius:=" << robot_radius << std::endl;
  std::cerr << "_voxelize_res:=" << voxelize_res << std::endl;
  std::cerr << "_angular_loss:=" << angular_loss << std::endl;
  std::cerr << "_max_angular_correction:=" << max_angular_correction << std::endl;
  std::cerr << "verbose:=" << verbose << std::endl;

  status_publisher =
    nh.advertise<srrg2_navigation_2d_msgs::CollisionAvoiderStatus>(status_topic, 10);
  cmd_vel_publisher          = nh.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 10);
  status_msg.header.frame_id = base_link_frame_id;
  ros::Subscriber cmd_vel_input_subscriber = nh.subscribe(cmd_vel_input_topic, 10, cmdVelCallback);
  ros::Subscriber scan_subscriber          = nh.subscribe(scan_topic, 10, scanCallback);
  ros::Rate loop_rate(50);
  listener.reset(new tf::TransformListener);
  ros::spin();
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
