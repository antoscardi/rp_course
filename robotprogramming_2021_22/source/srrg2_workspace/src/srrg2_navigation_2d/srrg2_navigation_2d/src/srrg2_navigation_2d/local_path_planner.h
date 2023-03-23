#pragma once
#include "obstacle_avoidance_base.h"
#include <algorithm>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/drawable_base.h>

#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool local_path_planner_debug = false;
namespace srrg2_navigation_2d {
  class LocalPathPlanner : public ObstacleAvoidanceBase, public srrg2_core::DrawableBase {
    using ThisType = LocalPathPlanner;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalPathPlanner();
    enum class Status { Success = 0x0, Error = 0x1 };
    inline Status status() const {
      return _status;
    }
    void compute() override;
    void setOrigin(const srrg2_core::Isometry2f& origin_);
    bool setPath(const srrg2_core::Point2fVectorCloud& points_,
                 srrg2_core::StdVectorEigenVector2f& targets_);
    void setGlobalGridMap(const srrg2_core::GridMap2D& global_grid_map_);

    inline void setDesiredTarget(const srrg2_core::Vector2i& target_pxl_) {
      _desired_target_pixel = target_pxl_;
      ObstacleAvoidanceBase::setDesiredTarget(_grid_map->indices2local(_desired_target_pixel));
      DEBUG(local_path_planner_debug)
        << "desired target pxl: " << _desired_target_pixel.transpose() << std::endl;
      DEBUG(local_path_planner_debug)
        << "desired target    : " << _desired_target.transpose() << std::endl;
    }

    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

    // bb global grid map getter
    inline srrg2_core::GridMap2D globalGridMap() const {
      return _global_grid_map;
    }

    inline srrg2_core::Vector2f desiredTargetInWorld() const {
      return _desired_target_in_world;
    }

  private:
    Status _status = Status::Error;
    bool _computePolicy(const srrg2_core::Point2iVectorCloud& goals_,
                        srrg2_core::PathMatrix& distance_map_);
    void _setDistanceMapForDijkstra();
    void _publishPath();
    bool _path_changed_flag                    = false;
    srrg2_core::Vector2i _desired_target_pixel = srrg2_core::Vector2i::Zero();
    srrg2_core::GridMap2D _global_grid_map;
    srrg2_core::Chrono::ChronoMap _chrono_map;
    srrg2_core::Isometry2f _origin;
    srrg2_core::Vector2f _desired_target_in_world = srrg2_core::Vector2f::Zero();
    srrg2_core::StdVectorEigenVector2i _targets_pxl;
  };

  using LocalPathPlannerPtr = std::shared_ptr<LocalPathPlanner>;
} // namespace srrg2_navigation_2d
