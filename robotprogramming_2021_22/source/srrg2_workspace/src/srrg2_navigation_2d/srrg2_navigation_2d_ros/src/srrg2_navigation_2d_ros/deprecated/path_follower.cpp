#include "path_follower.h"
#include "scan_handler.h"
#include <nav_msgs/Path.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg_converters/converter.h>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_viewer/viewer_canvas.h>

using namespace srrg2_core;
using namespace srrg2_navigation_2d;

#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool path_follower_debug = false;

namespace srrg2_navigation_2d_ros {
  PathFollower::PathFollower() {
    param_motion_controller->setStatusMsg(&_status_msg);
    _status = PathFollower::Status::Error;
  };

  void PathFollower::advertiseTopics(ros::NodeHandle& nh_) {
    // bb initialize the publishers
    _status_publisher = nh_.advertise<srrg2_navigation_2d_msgs::PathFollowerStatus>(
      param_path_follower_status.value(), 10);
    _local_path_publisher = nh_.advertise<nav_msgs::Path>(param_local_path_topic.value(), 10);
    _targets_publisher    = nh_.advertise<nav_msgs::Path>(param_target_topic.value(), 10);
  }

  void PathFollower::compute(GridMap2D& grid_map_,
                             const srrg2_core::Isometry2f& robot_in_world_,
                             const srrg2_core::Point2fVectorCloud& path_points_,
                             const ObstacleAvoidanceBase::Isometry2fList& path_poses_,
                             geometry_msgs::Twist& twist_) {
    // bb draw laser scan
    // param_scan_handler->draw(_canvas);
    _status_msg.header.frame_id  = param_base_link_frame_id.value();
    _status_msg.header.stamp     = ros::Time::now();
    Vector3f robot_pose_2d       = geometry2d::t2v(robot_in_world_);
    Vector3f goal_in_robot       = geometry2d::t2v(robot_in_world_.inverse() * path_poses_.back());
    _status_msg.robot_pose_2d[0] = robot_pose_2d.x();
    _status_msg.robot_pose_2d[1] = robot_pose_2d.y();
    _status_msg.robot_pose_2d[2] = robot_pose_2d.z();
    Point2fVectorCloud path_points_in_robot =
      path_points_.transform<TRANSFORM_CLASS::Isometry>(robot_in_world_.inverse());
    srrg2_core::StdVectorEigenVector2f targets;
    // bb set origin of local map
    param_local_path_planner->setOrigin(robot_in_world_);
    // bb set path to find the desired local goal
    if (!param_local_path_planner->setPath(path_points_, targets)) {
      std::cerr << FG_BCYAN("Was not able to set a local desired target from the intersection of "
                            "the global path with the local window!!!")
                << std::endl;
      std::cerr << FG_BMAGENTA("Please choose new goal!!!") << std::endl;
      _status_msg.status = "error";
      _status_publisher.publish(_status_msg);
      _status = PathFollower::Status::Error;
      return;
    }
    // bb set global grid map of local_path_planner if it has no valid value
    if (!param_local_path_planner->globalGridMap().GridMap2DHeader::size().x() ||
        !param_local_path_planner->globalGridMap().GridMap2DHeader::size().y()) {
      param_local_path_planner->setGlobalGridMap(grid_map_);
    }
    const Matrix_<float>& global_distance_map =
      grid_map_.property<srrg2_core::Property_<srrg2_core::Matrix_<float>>>("distances")->value();
    PathMessagePtr srrg_targets(new PathMessage);
    srrg_targets->topic.setValue("/target");
    srrg_targets->frame_id.setValue("/map");
    srrg_targets->timestamp.setValue(ros::Time::now().toSec());
    srrg_targets->seq.setValue(0);
    for (size_t i = 0; i < targets.size(); ++i) {
      PoseStampedMessage pose;
      pose.topic.setValue("/target");
      pose.frame_id.setValue("/map");
      pose.timestamp.setValue(ros::Time::now().toSec());
      pose.seq.setValue(i);
      srrg2_core::Vector3f target_with_orientation = srrg2_core::Vector3f::Zero();
      target_with_orientation << targets[i].x(), targets[i].y(), 0.f;
      pose.pose.value().setPose(target_with_orientation);
      srrg_targets->poses.pushBack(pose);
    }
    nav_msgs::PathPtr ros_targets = srrg2_core_ros::Converter::convert(srrg_targets);
    _targets_publisher.publish(ros_targets);
    std::cerr << FG_BBLUE("SIZE TARGETS: ") << targets.size() << std::endl;
    param_local_path_planner->setLaserScanPoints(param_scan_handler->scanPoints());
    // bb compute iff the robot is not doing the initial rotation
    param_local_path_planner->compute();
    const ObstacleAvoidanceBase::Isometry2fList& local_path = param_local_path_planner->path();
    PathMessagePtr srrg_local_path(new PathMessage);
    srrg_local_path->topic.setValue("/local_path");
    srrg_local_path->frame_id.setValue("base_link");
    srrg_local_path->timestamp.setValue(ros::Time::now().toSec());
    srrg_local_path->seq.setValue(0);
    const auto& local_path_vec =
      srrg2_core::StdVectorEigenIsometry2f(local_path.begin(), local_path.end());
    for (size_t i = 0; i < local_path_vec.size(); ++i) {
      PoseStampedMessage pose;
      pose.topic.setValue("/local_path");
      pose.frame_id.setValue("base_link");
      pose.timestamp.setValue(ros::Time::now().toSec());
      pose.seq.setValue(i);
      pose.pose.value().setPose(geometry2d::t2v(local_path_vec[i]));
      srrg_local_path->poses.pushBack(pose);
    }
    nav_msgs::PathPtr ros_local_path = srrg2_core_ros::Converter::convert(srrg_local_path);
    _local_path_publisher.publish(ros_local_path);
    std::cerr << FG_BBLUE("SIZE LOCAL PATH: ") << local_path.size() << std::endl;
    if (local_path.empty()) {
      std::cerr << FG_BMAGENTA("Just wait for the next laser scan to be received!") << std::endl;
      _status = PathFollower::Status::Error;
    } else {
      const Isometry2f& local_target_iso = *std::next(local_path.begin(), 5);
      Vector3f local_target_in_world     = geometry2d::t2v(robot_in_world_ * local_target_iso);
      float distance_to_global_tg        = (goal_in_robot.head<2>()).norm();
      param_motion_controller->setRobot(robot_pose_2d);
      param_motion_controller->setTarget(local_target_in_world);
      param_motion_controller->compute(twist_, distance_to_global_tg);
      DEBUG(path_follower_debug) << "Distance to global target: " << distance_to_global_tg
                                 << std::endl;
      if (param_motion_controller->status() == MotionController::Status::End) {
        _status_msg.status = "goal_reached";
        _status            = PathFollower::Status::End;
      } else if (param_motion_controller->status() == MotionController::Status::Rotating) {
        _status_msg.status = "initial_turning";
        _status            = PathFollower::Status::Rotating;
      } else {
        _status = PathFollower::Status::InProgress;
      }
    }
    _status_publisher.publish(_status_msg);
  }
} // namespace srrg2_navigation_2d_ros
