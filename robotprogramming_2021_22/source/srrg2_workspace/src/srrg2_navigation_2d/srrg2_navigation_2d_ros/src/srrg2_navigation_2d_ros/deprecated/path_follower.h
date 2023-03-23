#pragma once
#include "motion_controller.h"
#include "scan_handler.h"
#include <geometry_msgs/Twist.h>
#include <ros/publisher.h>
#include <srrg2_navigation_2d/local_path_planner.h>
#include <srrg_config/configurable.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_property/property.h>
namespace srrg2_core {
  class ViewerCanvas;
  using ViewerCanvasPtr = std::shared_ptr<ViewerCanvas>;
} // namespace srrg2_core

namespace srrg2_navigation_2d_ros {
  class PathFollower : public srrg2_core::Configurable {
  public:
    using ThisType = PathFollower;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum class Status { End = 0x0, InProgress = 0x1, Rotating = 0x2, Error = 0x1 };
    PARAM(srrg2_core::PropertyString, local_path_topic, "local path topic", "/local_path", nullptr);
    PARAM(srrg2_core::PropertyString,
          path_follower_status,
          "path follower status",
          "/path_follower_status",
          nullptr);
    PARAM(srrg2_core::PropertyString, target_topic, "target topic", "/target", nullptr);
    PARAM(srrg2_core::PropertyString,
          base_link_frame_id,
          "base link frame id",
          "/base_link",
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<ScanHandler>,
          scan_handler,
          "scan handler",
          std::shared_ptr<ScanHandler>(new ScanHandler()),
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<MotionController>,
          motion_controller,
          "motion controller",
          std::shared_ptr<MotionController>(new MotionController()),
          nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_navigation_2d::LocalPathPlanner>,
          local_path_planner,
          "local path planner",
          srrg2_navigation_2d::LocalPathPlannerPtr(new srrg2_navigation_2d::LocalPathPlanner()),
          nullptr);
    PathFollower();
    inline void setCanvas(srrg2_core::ViewerCanvasPtr canvas_) {
      _canvas = canvas_;
    }
    void advertiseTopics(ros::NodeHandle& nh_);
    void compute(srrg2_core::GridMap2D& grid_map_,
                 const srrg2_core::Isometry2f& robot_in_world_,
                 const srrg2_core::Point2fVectorCloud& path_points_,
                 const srrg2_navigation_2d::ObstacleAvoidanceBase::Isometry2fList& path_poses_,
                 geometry_msgs::Twist& twist_);
    inline PathFollower::Status status() const {
      return _status;
    }

  private:
    srrg2_core::ViewerCanvasPtr _canvas;
    ros::Publisher _status_publisher;
    ros::Publisher _local_path_publisher;
    ros::Publisher _targets_publisher;
    srrg2_navigation_2d_msgs::PathFollowerStatus _status_msg;
    PathFollower::Status _status;
  };

} // namespace srrg2_navigation_2d_ros
