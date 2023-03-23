#include <chrono>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_pcl/point_types.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg2_core_ros/PlannerStatusMessage.h>

#include <srrg2_navigation_2d_ros/tf_helpers.h>

const std::string exe_name("path_provider");
#define LOG std::cerr << srrg2_core::getTimestamp() + "|" + exe_name + "|"

using namespace srrg2_core;
using namespace std;

// ia load waypoints from file
using IDIsometry3fMap =
  std::map<int, Isometry3f, std::less<int>, Eigen::aligned_allocator<Isometry3f>>;
IDIsometry3fMap loadWaypointsFromFile(const std::string& filename_);
using PathDeque = std::deque<IDIsometry3fMap, Eigen::aligned_allocator<IDIsometry3fMap>>;

// ia global ros cmd line parameters
std::string waypoint_filename = ""; // ia where waypoints are stored
int path_iterations           = -1; // ia number of times you want to repeat path
std::string planner_status_topic("/planner_status"); // ia topic to listen
std::string nav_goal_topic("/move_base_simple/goal");            // ia topic where to publish

// ia very stupid flag that tells me when a waypoint has been reached
volatile bool goal_reached=false;

int num_target=0;

// gg: mamma che merda
inline void iso2rosPose(geometry_msgs::Pose& dest, const srrg2_core::Vector6f& src) {
  Eigen::Isometry3f iso = srrg2_core::geometry3d::v2t(src);
  Eigen::Quaternionf q(iso.linear());
  dest.position.x    = src(0);
  dest.position.y    = src(1);
  dest.position.z    = src(2);
  dest.orientation.x = q.x();
  dest.orientation.y = q.y();
  dest.orientation.z = q.z();
  dest.orientation.w = q.w();
}

inline void iso2rosPose(geometry_msgs::Pose& dest, const srrg2_core::Isometry3f& src) {
  iso2rosPose(dest, srrg2_core::geometry3d::t2v(src));
}

std::string previous_status;
void plannerStatusCallback(const srrg2_core_ros::PlannerStatusMessage& status_msg_) {
  // LOG << "current status is [ " << FG_YELLOW(status_msg_.status) << " ]\n";
  if (status_msg_.status!="Cruising" && previous_status!=status_msg_.status)
    std::cerr << srrg2_core::getTimestamp() + "|" + exe_name + "|current status is [ "
              << FG_YELLOW(status_msg_.status) << " ]" << std::endl;
  previous_status = status_msg_.status;
  if (status_msg_.status == "GoalReached"
             || status_msg_.status == "GoalNotAdmissible"
             || status_msg_.status == "GoalUnreachable") {
    goal_reached = true;
    ++num_target;
  } else {
    goal_reached=false;
  }
}

// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
IDIsometry3fMap loadWaypointsFromFile(const std::string& filename_) {
  if (!isAccessible(filename_)) {
    throw std::runtime_error("WaypointParser::loadWaypointsFromFile|ERROR, cannot access file [ " +
                             filename_ + " ]");
  }

  // ia return value
  IDIsometry3fMap waypoints;

  // ia open stream and start reading
  size_t num_lines = 0;
  std::string line;
  std::ifstream stream(filename_, std::ios::in);
  assert(stream.good() && "WaypointParser::loadWaypointsFromFile|ERROR, cannot open file");

  std::cerr << "loading waypoints: " << std::endl;
  while (std::getline(stream, line)) {
    // ia skip comments
    if (line.front() == '#') {
      continue;
    }

    // ia get the values
    Vector7f pose_vector = Vector7f::Zero();
    std::stringstream ss(line);

    // clang-format off
    ss >> pose_vector[0] // x
       >> pose_vector[1] // y
       >> pose_vector[2] // z
       >> pose_vector[3] // qx
       >> pose_vector[4] // qy
       >> pose_vector[5] // qz
       >> pose_vector[6];// qw
    // clang-format on

    // ia show waypoints
    LOG << "read waypoint $" << num_lines << " = [ " << pose_vector.transpose() << " ]\n";

    // ia accumulate
    Isometry3f waypoint_pose = srrg2_core::geometry3d::tqxyzq2t(pose_vector);
    waypoints.insert(std::make_pair(num_lines, waypoint_pose));

    // ia go forward in the file
    ++num_lines;
  }

  // ia consistency check
  if (num_lines != waypoints.size()) {
    throw std::runtime_error(exe_name + "|ERROR, something went wrong while reading the waypoints");
  }

  LOG << "read [ " << waypoints.size() << " ] waypoints from [ " << filename_ << " ]\n";

  return waypoints;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, exe_name);
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("waypoint_filename", waypoint_filename);
  _nh.getParam("path_iterations", path_iterations);
  _nh.getParam("status_topic", planner_status_topic);
  _nh.getParam("nav_goal_topic", nav_goal_topic);

  LOG << "running with these parameters" << std::endl;
  std::cerr << "\t_waypoint_filename:= " << waypoint_filename << std::endl;
  std::cerr << "\t_path_iterations:= " << path_iterations << std::endl;
  std::cerr << "\t_status_topic:= " << planner_status_topic << std::endl;
  std::cerr << "\t_nav_goal_topic:= " << nav_goal_topic << std::endl;

  // ia check cmd line parameters are good
  if (waypoint_filename.empty() || path_iterations < 1 || planner_status_topic.empty() ||
      nav_goal_topic.empty()) {
    throw std::runtime_error(exe_name + "|ERROR, invalid cmd line parameters");
  }

  // ia fuckin load waypoints -- those will constitue nav2dgoals
  const auto waypoints(std::move(loadWaypointsFromFile(waypoint_filename)));

  // ia now we have to listen to the status of the path_follower node.
  // ia when the status is "goal_reached" we wait a bit a supply another waypoint.
  // ia when we finish the waypoints, we restart - another iteration of the same waypoints

  // ia subscribe to the path follower to get the status
  ros::Subscriber path_follower_status_subscriber =
    nh.subscribe(planner_status_topic, 10, plannerStatusCallback);

  // ia advertise a waypoints (aka navigation goals) on topic
  geometry_msgs::PoseStamped current_waypoint_pose_msg;
  current_waypoint_pose_msg.header.frame_id = "map";
  ros::Publisher waypoints_publisher = nh.advertise<geometry_msgs::PoseStamped>(nav_goal_topic, 10);

  // ia current trajectory iteration
  int remaining_path_iteration         = path_iterations;
  size_t current_waypoint_idx          = 0;
  bool first_waypoint                  = false;
  size_t current_waypoint_pose_msg_seq = 0;

  // ia start looping at custom rate
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    // ia if it's the first ever waypoint that we have to publish, wait for a user command
    if (remaining_path_iteration == path_iterations && current_waypoint_idx == 0) {
      LOG << "first waypoint has to be published, press [ "
          << "ENTER"
          << " ] to publish it!\n";
      std::cin.get();
      first_waypoint = true;
    }

    // ia here we should publish stuff
    if (goal_reached || first_waypoint) {
      LOG << "new waypoint ready [ " << current_waypoint_idx << " ] " << std::endl;

      // ia wait a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      // ia reset flag
      goal_reached = false;

      // ia check if current path is ended and reset waypoint idx
      if (current_waypoint_idx >= waypoints.size()) {
        --remaining_path_iteration;
        current_waypoint_idx = 0;
      }

      // ia check if path iterations if ended
      if (remaining_path_iteration == 0) {
        LOG << "path iterations ended -- "
            << "stop!" << std::endl;
        break;
      }

      // ia get next waypoint pose
      const auto& current_waypoint_pose = waypoints.at(current_waypoint_idx);
      // LOG << "waypoint pose\n" << current_waypoint_pose.matrix() << std::endl;

      // ia publish
      LOG << "path iteration [ " << path_iterations - remaining_path_iteration << "/"
          << path_iterations - 1 << " ] -- waypoint [ " << current_waypoint_idx << "/"
          << waypoints.size() - 1 << " ] published!\n";
      current_waypoint_pose_msg.header.seq   = current_waypoint_pose_msg_seq++;
      current_waypoint_pose_msg.header.stamp = ros::Time::now();
      iso2rosPose(current_waypoint_pose_msg.pose, current_waypoint_pose);
      waypoints_publisher.publish(current_waypoint_pose_msg);

      // ia go forward in the current path
      ++current_waypoint_idx;
      first_waypoint = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
