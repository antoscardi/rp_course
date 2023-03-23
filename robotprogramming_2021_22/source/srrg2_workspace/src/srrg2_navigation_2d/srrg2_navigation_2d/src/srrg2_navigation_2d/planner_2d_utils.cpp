#include "planner_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_config/configurable_command.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/messages/pose_array_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/drawable_base.h>
#include <unistd.h>
#include <srrg_messages/messages/planner_status_message.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  void Planner2D::getMapHeader(GridMap2DHeader& map_header_, const Vector3f& robot_pose_) {
    int lm_radius_pixel      = param_local_map_radius.constValue() / _grid_map->resolution();
    size_t lm_diameter_pixel = lm_radius_pixel * 2;
    Vector2i lm_center(lm_radius_pixel, lm_radius_pixel);
    Vector2i lm_upper_left = _grid_map->global2indices(robot_pose_.head<2>()) - lm_center;
    map_header_.setSize(Vector2i(lm_diameter_pixel, lm_diameter_pixel));
    map_header_.setResolution(_grid_map->resolution());
    // bb compute origin translation vector
    Vector2f origin_translation = _grid_map->indices2global(lm_upper_left) + map_header_.center();
    Isometry2f origin;
    origin.translation() = origin_translation;
    origin.linear()      = Matrix2f::Identity();
    map_header_.setOrigin(origin);
  }

  void Planner2D::pixelizeScanPoints(Point2iVectorCloud& scan_pts_pxl_,
                                     const Point2fVectorCloud& scan_pts_,
                                     const GridMap2DHeader& map_header_) {
    scan_pts_pxl_.resize(scan_pts_.size());

    for (const auto& point : scan_pts_) {
      // bb assume resolution of local and global map is the same
      Vector2i indices = map_header_.global2indices(point.coordinates());
      Point2i point_idx;
      point_idx.coordinates() = indices;
      scan_pts_pxl_.push_back(std::move(point_idx));
    }
  }


} // namespace srrg2_navigation_2d
