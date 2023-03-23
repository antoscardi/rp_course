#include "motion_controller.h"
// bb debug print
#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool motion_controller_debug = false;
namespace srrg2_navigation_2d_ros {
  bool MotionController::compute(geometry_msgs::Twist& twist_,
                                 const Eigen::Vector3f& delta_,
                                 bool finalize_ = false) {
    if (!_status_msg) {
      return false;
    }
    twist_.linear.x  = 0;
    twist_.linear.y  = 0;
    twist_.linear.z  = 0;
    twist_.angular.x = 0;
    twist_.angular.y = 0;
    twist_.angular.z = 0;

    // std::cerr << "c";
    Eigen::Vector2f t = delta_.head<2>();
    float theta       = delta_.z();
    // goal reached condition
    // handle goal behind the target with pure rotation
    float theta_to_point = atan2(t.y(), t.x());
    DEBUG(motion_controller_debug) << "THETA_TO_POINT: " << theta_to_point << std::endl;
    DEBUG(motion_controller_debug) << "DISTANCE TARGET ROBOT: " << t.norm() << std::endl;
    if (fabs(theta_to_point) > param_pure_rotation_threshold.value() &&
        t.norm() > param_translation_reach_threshold.value()) {
      twist_.angular.z    = param_rv_gain.value() * theta_to_point;
      _status_msg->status = "initial_turning";
      return true;
    }

    if (t.norm() > param_translation_reach_threshold.value()) {
      twist_.linear.x     = t.norm() * param_tv_gain.value();
      twist_.angular.z    = twist_.linear.x * 2 * t.y() / t.squaredNorm();
      _status_msg->status = "cruising";
      return true;
    }

    if (finalize_ && fabs(theta) > param_rotation_reach_threshold.value()) {
      twist_.angular.z    = param_rv_gain.value() * theta;
      _status_msg->status = "finalizing";
      return true;
    }

    return false;
  };

  void MotionController::compute(geometry_msgs::Twist& twist_,
                                 const float& distance_to_global_target_) {
    // bb compute useful quantities
    const float& theta                       = _robot_pose_vector(2);
    const float& theta_target                = _target(2);
    srrg2_core::Vector2f robot_sagittal_axis = srrg2_core::Vector2f::Zero();
    robot_sagittal_axis << cos(theta), sin(theta);
    srrg2_core::Vector2f robot_coronal_axis = srrg2_core::Vector2f::Zero();
    robot_coronal_axis << -sin(theta), cos(theta);
    srrg2_core::Vector2f error_vector = srrg2_core::Vector2f::Zero();
    error_vector << _target.head<2>() - _robot_pose_vector.head<2>();
    const float theta_error              = atan2(error_vector.y(), error_vector.x());
    srrg2_core::Vector2f target_velocity = srrg2_core::Vector2f::Zero();
    target_velocity << cos(theta_target), sin(theta_target);
    float pointing_error = theta_error - theta;
    pointing_error       = atan2(sin(pointing_error), cos(pointing_error));
    DEBUG(motion_controller_debug) << "Pointing error: " << pointing_error << std::endl;
    // bb arrived
    if (distance_to_global_target_ <= 0.2f) {
      twist_.linear.x  = 0.f;
      twist_.angular.z = 0.f;
      _status          = Status::End;
    } else if (robot_sagittal_axis.dot(target_velocity) < 0) {
      // bb rotating on place to reorient
      twist_.linear.x  = 0.f;
      twist_.angular.z = param_rv_gain.value() * pointing_error;
      _status          = Status::Rotating;
    } else {
      // bb compute linear and angular controls
      twist_.linear.x = param_k1.value() * error_vector.dot(robot_sagittal_axis) +
                        param_kb.value() * target_velocity.dot(robot_sagittal_axis);
      twist_.angular.z =
        param_k2.value() * pointing_error + param_k3.value() * error_vector.dot(robot_coronal_axis);
      _status = Status::Travelling;
    }
  }
} // namespace srrg2_navigation_2d_ros
