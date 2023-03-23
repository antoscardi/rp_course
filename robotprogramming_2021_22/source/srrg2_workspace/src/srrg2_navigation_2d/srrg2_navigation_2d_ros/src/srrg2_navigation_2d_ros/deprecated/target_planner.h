#pragma once
#include "Eigen/Geometry"
#include "srrg_pcl/point_types.h"
#include "tf_helpers.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <srrg2_laser_slam_2d/sensor_processing/raw_data_preprocessor_projective_2d.h>
#include <srrg2_navigation_2d_msgs/CollisionAvoiderStatus.h>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_data_structures/path_matrix_dijkstra_search.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_pcl/normal_computator.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/drawable_base.h>
#include <srrg_viewer/viewer_manager_shared.h>
using namespace srrg2_core;

class TargetPlanner : public Configurable, public DrawableBase {
  using PropertyCostType            = Property_<Matrix_<float>>;
  using PointNormal2fVectorCloudPtr = std::shared_ptr<PointNormal2fVectorCloud>;
  using ThisType                    = TargetPlanner;
  using Vector2fVector              = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;

public:
  enum Status { Success = 0x0, FarFromObstacles = 0x1, Error = 0x2 };
  enum Type { Distance = 0x0, Cost = 0x1 };
  // bb algorithms to be used for building the distance and the cost map
  PARAM(srrg2_core::PropertyConfigurable_<PathMatrixDistanceSearch>,
        distance_map_computator,
        "algorithm to compute the distance map",
        PathMatrixDistanceSearchPtr(new PathMatrixDistanceSearch()),
        nullptr);
  PARAM(srrg2_core::PropertyConfigurable_<PathMatrixDijkstraSearch>,
        cost_map_computator,
        "algorithm to compute the cost map",
        PathMatrixDijkstraSearchPtr(new PathMatrixDijkstraSearch()),
        nullptr);
  PARAM(PropertyFloat, robot_radius, "Robot radius", 0.2, nullptr);
  PARAM(PropertyFloat, voxelize_res, "voxelize res", 0.01, nullptr);
  PARAM(PropertyUnsignedInt, dim_local_window, "local map dimension [pixel]", 100, nullptr);
  PARAM(PropertyBool, verbose, "verbose", true, nullptr);
  TargetPlanner() {
    _status                  = ThisType::Status::Error;
    offset                   = Vector2f::Ones() * param_dim_local_window.value();
    float robot_radius_pixel = param_robot_radius.value() / param_voxelize_res.value();
    std::cerr << "creating cost layers" << std::endl;
    int rows = offset.y() * 2;
    int cols = offset.x() * 2;
    Vector2i size;
    size << rows, cols;
    _grid_map = GridMap2DPtr(new GridMap2D);
    _grid_map->setSize(size);
    _grid_map->setResolution(param_voxelize_res.value());
    // bb as we are referring to the robot, our local map will have center at
    // Eigen::Isometry2f::Identity() which is its default value
    // installs 3 new layers
    PropertyCostType* prop_cost = _grid_map->property<PropertyCostType>("cost_map");
    if (!prop_cost) {
      prop_cost =
        new PropertyCostType("cost_map", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
    }
    prop_cost->value().fill(-1.f);

    PropertyCostType* prop_cost_dx = _grid_map->property<PropertyCostType>("cost_map_dx");
    if (!prop_cost_dx) {
      prop_cost_dx = new PropertyCostType(
        "cost_map_dx", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
    }
    prop_cost_dx->value().fill(0.f);

    PropertyCostType* prop_cost_dy = _grid_map->property<PropertyCostType>("cost_map_dy");
    if (!prop_cost_dy) {
      prop_cost_dy = new PropertyCostType(
        "cost_map_dy", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
    }
    prop_cost_dy->value().fill(0.f);
    if (param_voxelize_res.value() == 0) {
      throw std::runtime_error("voxelize_res zero!!!");
    }
    if (rows == 0 || cols == 0) {
      throw std::runtime_error("Empty distance map!!!");
    }
    _distance_map.resize(rows, cols);
    param_distance_map_computator->setPathMatrix(&_distance_map);
    param_distance_map_computator->param_max_distance_squared_pxl.setValue(4 * robot_radius_pixel *
                                                                           robot_radius_pixel);
    param_cost_map_computator->param_min_distance.setValue(0.8 * robot_radius_pixel *
                                                           robot_radius_pixel);
  }
  const Vector2f& desiredTarget() const {
    return _desired_target;
  }
  const Vector2f& actualTarget() const {
    return _actual_target;
  }
  void setLaserScanPoints(const PointNormal2fVectorCloud& points_) {
    _laser_scan_points = points_;
  }
  void setDesiredTarget(const Vector2f& target_) {
    _desired_target = target_;
  }
  inline Status status() const {
    return _status;
  }
  void compute();
  void reset();
  void _drawImpl(srrg2_core::ViewerCanvasPtr canvas) const override;

protected:
  void computeDistanceMap(const Point2iVectorCloud& pixel_point_cloud_);
  void plotDistanceMap();
  void plotDistanceMap(const PathMatrix& distance_map,
                       const Vector2i& target,
                       const Vector2fVector& path);
  Vector2f computeGradientDistanceMap(const PathMatrix& distance_map,
                                      const Vector2i& parent_cell_,
                                      const Type);
  Vector2i
  modifyTarget(const Vector2i& target_pixel_, float squared_distance_, const Vector2f& gradient_);
  bool computePolicy(const Vector2i& goal_, PathMatrix& distance_map);
  bool computePathGradient(const Vector2f& current_pose);
  void updateCostLayers(const PathMatrix& distance_map) const;

private:
  PointNormal2fVectorCloud _laser_scan_points;
  Vector2f offset = Vector2f::Zero();

  Status _status = Error;
  std::vector<Vector2i> _path_indexes;
  PathMatrix _distance_map;
  srrg2_core::GridMap2DPtr _grid_map = nullptr;
  Vector2f _gradient_dijkstra        = Vector2f::Zero();
  Vector2f _gradient_distance_map    = Vector2f::Zero();
  Vector2f _desired_target           = Vector2f::Zero();
  Vector2f _actual_target            = Vector2f::Zero();
  Vector2fVector _path;
  // only for debug
  ImageUInt8 _path_map_image;
  ImageInt _parent_map_image;
  ImageUInt8 _cost_map_image;
  Vector2f _desired_target_parent = Vector2f::Zero();
  cv::Mat _m, _m_scaled, _p, _p_scaled, _q, _q_scaled;
  Vector2f _actual_target_draw = Vector2f::Zero();
  // Vector2i _desired_target_parent;
  // Vector2i _desired_target_pixel;
};

using TargetPlannerPtr = std::shared_ptr<TargetPlanner>;
