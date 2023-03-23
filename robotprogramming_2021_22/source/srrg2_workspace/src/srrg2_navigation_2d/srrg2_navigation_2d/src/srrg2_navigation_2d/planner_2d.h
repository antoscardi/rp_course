#pragma once
#include "navigation_2d_base.h"
#include <srrg_data_structures/path_matrix_cost_search.h>
#include <srrg_data_structures/path_matrix_dijkstra_search.h>
#include <srrg_property/property_eigen.h>
#include <srrg_viewer/active_drawable.h>
namespace srrg2_navigation_2d {
  using namespace srrg2_core;

  class Planner2D : public Navigation2DBase, public ActiveDrawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum PathType { Gradient = 0x0, Grid = 0x1 };
    enum PlannerStatus {
      Idle              = 0,
      GoalNotAdmissible = 1,
      GoalUnreachable   = 2,
      GlobalPlanning    = 3,
      Cruising          = 4,
      GoalReached       = 5
    };

    using PropertyCostType = Property_<Matrix_<float>>; // float matrix
    PARAM(PropertyFloat, range_min, "range_min [meters]", 0.1, nullptr);

    PARAM(PropertyFloat, range_max, "range_max [meters]", 5.0, nullptr);

    PARAM(PropertyFloat,
          grid_gain,
          "gain for plain dijkstra when computing interpolation",
          20.0,
          nullptr);

    PARAM(PropertyFloat, goal_reach_distance, "below that, the goal is reached", 0.7, nullptr);

    PARAM(PropertyDouble, scan_update_time, "min_update_time [seconds]", 0.8, nullptr);

    PARAM(PropertyString, odom_topic, "topic where to read the odometry", "/odom", nullptr);

    PARAM(PropertyVector_<float>,
          cost_coeffs,
          "cost = p0 * distance_to_neighbor + sum_i p_i / (dist-robot_radius)",
          std::vector<float>(),
          &_cost_map_changed_flag);

    PARAM(PropertyString, goal_pose_topic, "move_base_simple/goal", "", nullptr);

    PARAM(PropertyFloat,
          scan_voxelize_resolution,
          "subsamples the laser endpoints for efficiency, 0: disables",
          0.05,
          nullptr);

    PARAM(PropertyFloat, local_map_radius, " tumadre ", 3.f, nullptr);
    PARAM(PropertyInt, path_type, "type of path, 0: gradient, 1 grid", PathType::Gradient, nullptr);

    Planner2D();

    // shell commands with response
    bool cmdSetGoal(std::string& response, float x, float y, float theta);

  protected:
    bool handleSetGoal(BaseSensorMessagePtr msg_);
    bool handleCostMapChanged();
    bool handleScan(BaseSensorMessagePtr msg_);
    bool computePolicy(const Vector3f& goal);
    bool putMessage(BaseSensorMessagePtr msg_) override;
    PathMatrixCostSearch::SearchStatus
    doDijkstra(const Vector2i& goal, const Vector2i& start, PathMatrix& pmap);

    float unrollPathPixel(StdVectorEigenVector3f& dest,
                          const StdVectorEigenVector2i& src,
                          const GridMap2DHeader& map_header_);
    void getMapHeader(GridMap2DHeader& map_header_, const Vector3f& robot_pose_);
    void pixelizeScanPoints(Point2iVectorCloud& scan_pts_pxl_,
                            const Point2fVectorCloud& scan_pts_,
                            const GridMap2DHeader& map_header_);
    void updateLocalDistanceMap(PathMatrix& local_dmap_,
                                GridMap2DHeader& map_header_,
                                const PathMatrix& global_dmap_,
                                const Vector3f& robot_pose_,
                                const Point2fVectorCloud& scan_pts_);
    void updateGlobalDistanceMap(const Point2fVectorCloud& scan_pts_);
    void updateGlobalPath();

    // in planner_2d_path.cpp
    void
    publishPath(const std::string& topic, const StdVectorEigenVector3f& path, double timestamp = 0);
    void publishStatus(bool anytime = false);
    float computePathGradient(StdVectorEigenVector3f& path_,
                              const Vector3f& current_pose,
                              PathMatrix& pm_,
                              const GridMap2DHeader& mh_);
    float computePathGrid(StdVectorEigenVector3f& path_,
                          const Vector3f& current_pose_,
                          PathMatrix& pm_,
                          const GridMap2DHeader& mh_);

    float computePathAStar(StdVectorEigenVector3f& path_,
                           const Vector3f& current_pose_,
                           PathMatrix& pm_,
                           const GridMap2DHeader& mh_);
    void setMap(GridMap2DPtr grid_map) override;

    void _drawImpl(ViewerCanvasPtr gl_canvas_) const override;
    // bb time of last publishing
    double _last_scan_update_time   = 0;
    double _last_goal_update_time   = 0;
    double _last_status_update_time = 0;
    bool _cost_map_changed_flag     = false;
    Vector3f _goal                  = Eigen::Vector3f::Zero();
    PathMatrixDijkstraSearch _dijkstra_search;
    StdVectorEigenVector3f _path;
    StdVectorEigenVector3f _local_path;
    PathMatrix _local_pmap, _global_pmap;
    // bb debug
    cv::Mat _shown_image;
    PlannerStatus _planner_status = Idle;
    float _cost_to_global_goal_msg;
    float _distance_to_global_goal;
    float _distance_to_local_goal;
    PathType _local_path_type;
    PathMatrixCostSearch::SearchStatus _a_star_status;
    Vector3f _current_robot_pose;
    void updateDistance();
    int _seq = 0;
  };
  using Planner2DPtr = std::shared_ptr<Planner2D>;
} // namespace srrg2_navigation_2d
