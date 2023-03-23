#pragma once
#include "srrg_pcl/point_types.h"
#include "target_planner.h"
#include "tf_helpers.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_laser_slam_2d/sensor_processing/raw_data_preprocessor_projective_2d.h>
#include <srrg2_navigation_2d_msgs/CollisionAvoiderStatus.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/drawable_base.h>
#include <srrg_viewer/viewer_manager_shared.h>
using namespace srrg2_core;

class Controller : public Configurable, public DrawableBase {
public:
  PARAM(srrg2_core::PropertyConfigurable_<srrg2_laser_slam_2d::RawDataPreprocessorProjective2D>,
        raw_data_preprocessor,
        "raw data preprocessor to get PointNormal2fVactorCloud from scan",
        srrg2_laser_slam_2d::RawDataPreprocessorProjective2DPtr(
          new srrg2_laser_slam_2d::RawDataPreprocessorProjective2D()),
        nullptr);
  PARAM(PropertyFloat, pure_rotation_threshold, "Pure Rotation Threshold", M_PI / 4, nullptr);
  PARAM(PropertyFloat, rotation_reach_threshold, "Pure Rotation Threshold", M_PI / 16, nullptr);
  PARAM(PropertyFloat, translation_reach_threshold, "Pure Translation Threshold", 0.5, nullptr);
  PARAM(PropertyFloat, rv_gain, "rv gain", 1, nullptr);
  PARAM(PropertyFloat, tv_gain, "tv gain", 1, nullptr);
  Controller(srrg2_navigation_2d_msgs::CollisionAvoiderStatus& status_msg,
             const ros::Publisher& cmd_vel_publisher,
             const ros::Publisher& status_publisher,
             const std::string& scan_topic) :
    _status_msg{status_msg},
    _cmd_vel_publisher{cmd_vel_publisher},
    _status_publisher{status_publisher} {
    _status_msg.header.frame_id = base_link_frame_id;
    param_raw_data_preprocessor->param_scan_topic.setValue(scan_topic);
    _target_planner = TargetPlannerPtr(new TargetPlanner);
  }
  void cmdVelCallback(const geometry_msgs::Twist& twist_input);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  TargetPlannerPtr _target_planner = nullptr;

protected:
  void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;

private:
  bool computeControl(geometry_msgs::Twist& twist, const Eigen::Vector3f& delta);
  void computeTarget(const geometry_msgs::Twist& twist_input_);
  std::string base_link_frame_id = "/base_link";
  PointNormal2fVectorCloud _scan_points_all_normal;
  srrg2_navigation_2d_msgs::CollisionAvoiderStatus& _status_msg;
  const ros::Publisher& _cmd_vel_publisher;
  const ros::Publisher& _status_publisher;
  double _tic                   = 0;
  double _cmdVelCallback_period = 0;
  double _max_cmd_vel_callback  = 0;
  double _minimum_period        = std::numeric_limits<double>::max();
  double _prev_tic              = 0;
  int _count                    = 0;
  double _period                = 0;
  const float _input_scaling    = 0.5;
  Vector2f _desired_target      = Vector2f::Zero();
};
