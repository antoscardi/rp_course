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
#include <srrg_viewer/viewer_manager_shared.h>
using namespace srrg2_core;
// hardcoded parameters
// float action_force_range         = 0.5;

class CallbackHandler : public Configurable {
  using PropertyCostType = Property_<Matrix_<float>>;

public:
  PARAM(srrg2_core::PropertyConfigurable_<srrg2_laser_slam_2d::RawDataPreprocessorProjective2D>,
        raw_data_preprocessor,
        "raw data preprocessor to get PointNormal2fVactorCloud from scan",
        srrg2_laser_slam_2d::RawDataPreprocessorProjective2DPtr(
          new srrg2_laser_slam_2d::RawDataPreprocessorProjective2D()),
        nullptr);
  CallbackHandler(const ViewerCanvasPtr& canvas,
                  srrg2_navigation_2d_msgs::CollisionAvoiderStatus& status_msg,
                  const ros::Publisher& cmd_vel_publisher,
                  const ros::Publisher& status_publisher,
                  const std::string& scan_topic) :
    _canvas{canvas},
    _listener{new tf::TransformListener},
    _status_msg{status_msg},
    _cmd_vel_publisher{cmd_vel_publisher},
    _status_publisher{status_publisher} {
    _status_msg.header.frame_id = base_link_frame_id;
    param_raw_data_preprocessor->param_scan_topic.setValue(scan_topic);
    std::cerr << "creating cost layers" << std::endl;
    int rows = offset.y() * 2;
    int cols = offset.x() * 2;
    // installs 3 new layers
    // PropertyCostType* prop_cost = _grid_map->property<PropertyCostType>("cost_map");
    // if (!prop_cost) {
    //   prop_cost =
    //     new PropertyCostType("cost_map", "", _grid_map.get(), Matrix_<float>(rows, cols),
    //     nullptr);
    // }
    // prop_cost->value().fill(-1.f);

    // PropertyCostType* prop_cost_dx = _grid_map->property<PropertyCostType>("cost_map_dx");
    // if (!prop_cost_dx) {
    //   prop_cost_dx = new PropertyCostType(
    //     "cost_map_dx", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
    // }
    // prop_cost_dx->value().fill(0.f);

    // PropertyCostType* prop_cost_dy = _grid_map->property<PropertyCostType>("cost_map_dy");
    // if (!prop_cost_dy) {
    //   prop_cost_dy = new PropertyCostType(
    //     "cost_map_dy", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
    // }
    // prop_cost_dy->value().fill(0.f);
    if (voxelize_res == 0) {
      throw std::runtime_error("voxelize_res zero!!!");
    }
    float robot_radius_pixel = robot_radius / voxelize_res;
    if (rows == 0 || cols == 0) {
      throw std::runtime_error("Empty distance map!!!");
    }
    _distance_map.resize(rows, cols);
    _dmap_calculator.setPathMatrix(&_distance_map);
    _dmap_calculator.param_max_distance_squared_pxl.setValue(4 * robot_radius_pixel *
                                                             robot_radius_pixel);

    std::cerr << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
    std::cerr << "_robot_radius:=" << robot_radius << std::endl;
    std::cerr << "_voxelize_res:=" << voxelize_res << std::endl;
    std::cerr << "_angular_loss:=" << angular_loss << std::endl;
    std::cerr << "_max_angular_correction:=" << max_angular_correction << std::endl;
    std::cerr << "verbose:=" << verbose << std::endl;
  }
  void cmdVelCallback(const geometry_msgs::Twist& twist_input);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
  void draw();
  void draw_in_cmd_vel_callback();

protected:
  static Point2iVectorCloud toPixel(const PointNormal2fVectorCloud& points_);
  static Vector2i toPixel(const Vector2f& point_);
  Vector2f fromPixel(const Vector2i& pixel_);
  void computeDistanceMap(Point2iVectorCloud pixel_point_cloud_);
  void plotDistanceMap();
  void updateCostLayers() const;
  bool computePolicy(const Vector2i& goal_);
  Vector2f computeGradientDijkstra();
  Vector3f applyGradient(const Vector2f& gradient_);
  Vector2i applyGradient(const Vector2f& gradient_, bool b);
  bool computeControl(geometry_msgs::Twist& twist, const Eigen::Vector3f& delta);
  PointNormal2f pixelToWorld(PathMatrixCell* desired_target_parent_,
                             const Point2iVectorCloud pixel_point_cloud_);
  PointNormal2fVectorCloud computeNormal(const Point2fVectorCloud& point_cloud_);
  Vector2f computeGradientDistanceMap(PathMatrixCell* parent_cell_);
  Vector2f computeGradientDistanceMap(const Vector2i& parent_cell_);
  void computeTarget(const geometry_msgs::Twist& twist_input_);
  void modifyTarget(const PointNormal2f& parent_in_world_, float distance_);
  Vector2i
  modifyTarget(Vector2i& target_pixel_, float squared_distance_, const Vector2f& gradient_);
  // void applyNextDirection(geometry_msgs::Twist& twist_output_, const Vector2f&
  // next_direction_);

private:
  bool scan2points(const sensor_msgs::LaserScan& scan);
  ViewerCanvasPtr _canvas = nullptr;
  PointNormal2fVectorCloud _scan_points_all_normal;
  // only for debugging
  std::shared_ptr<tf::TransformListener> _listener = nullptr;
  ImageUInt8 _path_map_image;
  ImageInt _parent_map_image;
  ImageUInt8 _cost_map_image;
  std::vector<Vector2i> _path_indexes;
  PathMatrix _distance_map;
  PathMatrixDistanceSearch _dmap_calculator;
  PathMatrixDijkstraSearch _path_search;
  srrg2_core::GridMap2DPtr _grid_map = nullptr;
  cv::Mat _m, _m_scaled, _p, _p_scaled, _q, _q_scaled;
  Vector2f _desired_target;
  Vector2f _actual_target;
  Vector2f _gradient_dijkstra;
  Vector2f _gradient_distance_map;
  PointNormal2f _desired_target_parent_with_normal;
  std::string base_link_frame_id      = "/base_link";
  float robot_radius                  = 0.2 /*0.2*/;
  static constexpr float voxelize_res = 0.01;
  float angular_loss                  = 10;
  float max_angular_correction        = 0.5;
  bool verbose                        = false;
  float linear_velocity_min           = 1e-2;
  double last_scan_stamp              = 0;
  float pure_rotation_threshold       = M_PI / 4;
  float rotation_reach_threshold      = M_PI / 16;
  float translation_reach_threshold   = 0.5;
  float rv_gain                       = 1;
  float tv_gain                       = 1;
  const float input_scaling           = 0.5;
  static Vector2f offset;
  double _tic;
  double _cmdVelCallback_period;
  srrg2_navigation_2d_msgs::CollisionAvoiderStatus& _status_msg;
  const ros::Publisher& _cmd_vel_publisher;
  const ros::Publisher& _status_publisher;
  double _max_cmd_vel_callback = 0;
  double _minimum_period       = std::numeric_limits<double>::max();
  double _prev_tic             = 0;
  int _count                   = 0;
  double _period               = 0;
  // Vector2i _desired_target_parent;
  // Vector2i _desired_target_pixel;
};
