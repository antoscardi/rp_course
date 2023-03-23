#pragma once
#include <geometry_msgs/Twist.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg_config/configurable.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_property/property.h>

namespace srrg2_navigation_2d_ros {
  class MotionController : public srrg2_core::Configurable {
    using thisType = MotionController;

  public:
    enum class Status { Travelling = 0x0, Rotating = 0x1, Error = 0x2, End = 0x3 };
    inline Status status() const {
      return _status;
    }
    PARAM(srrg2_core::PropertyFloat,
          pure_rotation_threshold,
          "pure rotation threshold",
          M_PI / 4,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          rotation_reach_threshold,
          "rotation reach threshold",
          M_PI / 16,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          translation_reach_threshold,
          "translation reach threshold",
          0.005,
          nullptr);
    PARAM(srrg2_core::PropertyFloat, rv_gain, "rotation velocity gain", 1.0f, nullptr);
    PARAM(srrg2_core::PropertyFloat, tv_gain, "translation velocity gain", 1.0f, nullptr);
    PARAM(srrg2_core::PropertyFloat, k1, "linear velocity feedback gain", 0.3f, nullptr);
    PARAM(srrg2_core::PropertyFloat, kb, "linear velocity feedforward gain", 0.5f, nullptr);
    PARAM(srrg2_core::PropertyFloat, k2, "angular velocity feedback gain 1", 0.1f, nullptr);
    PARAM(srrg2_core::PropertyFloat, k3, "angular velocity feedback gain 2", 0.1f, nullptr);
    MotionController() = default;
    bool compute(geometry_msgs::Twist& twist_, const Eigen::Vector3f& delta_, bool finalize_);
    void compute(geometry_msgs::Twist& twist_, const float& distance_to_global_target_);
    void setTarget(const srrg2_core::Vector3f& target_) {
      _target = target_;
    }
    void setRobot(const srrg2_core::Vector3f& robot_) {
      _robot_pose_vector = robot_;
    }
    void setStatusMsg(srrg2_navigation_2d_msgs::PathFollowerStatus* status_msg_) {
      _status_msg = status_msg_;
    }

  private:
    srrg2_navigation_2d_msgs::PathFollowerStatus* _status_msg = nullptr;
    srrg2_core::Vector3f _target                              = srrg2_core::Vector3f::Zero();
    srrg2_core::Vector3f _robot_pose_vector                   = srrg2_core::Vector3f::Zero();
    Status _status                                            = Status::Error;
  };
} // namespace srrg2_navigation_2d_ros
