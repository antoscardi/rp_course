#include "planner_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/planner_status_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <unistd.h>
#include "planner_2d_helpers.h"

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  void Planner2D::updateLocalDistanceMap(PathMatrix& local_dmap_,
                                         GridMap2DHeader& map_header_,
                                         const PathMatrix& global_dmap_,
                                         const Vector3f& robot_pose_,
                                         const Point2fVectorCloud& scan_pts_) {
    getMapHeader(map_header_, robot_pose_);
    Point2iVectorCloud scan_pts_pxl;
    pixelizeScanPoints(scan_pts_pxl, scan_pts_, map_header_);
    Vector2i lm_upper_left =
      _grid_map->global2indices(robot_pose_.head<2>()) - map_header_.local2indices(Vector2f(0, 0));
    Vector2i lm_lower_right =
      _grid_map->global2indices(robot_pose_.head<2>()) + map_header_.local2indices(Vector2f(0, 0));

    // bb copy portion of distance map
    // bb distance field will be combined with the current laser scan
    // bb cost will be the heuristic
    // bb parent not really needed
    local_dmap_.clear();
    global_dmap_.copyRegionTo(
      local_dmap_, lm_upper_left.x(), lm_lower_right.x(), lm_upper_left.y(), lm_lower_right.y());

    // bb add current laser scan to distance map
    PathMatrixDistanceSearch dmap_computator;
    dmap_computator.setPathMatrix(&_local_pmap);
    int map_size = min(map_header_.size().x(), map_header_.size().y());
    dmap_computator.param_max_distance_squared_pxl.setValue(map_size * map_size);
    dmap_computator.setGoals(scan_pts_pxl, false);
    dmap_computator.compute();
    ImageUInt8 dest;
    _local_pmap.toImage(dest, PathMatrix::Distance);
    dest.toCv(_shown_image);
    if (this->_canvases.size()) {
      paintPoints<Point2iVectorCloud::iterator, CoordinatesAccessor_<Point2iVectorCloud::iterator>>(
        _shown_image, scan_pts_pxl.begin(), scan_pts_pxl.end());
    }
  }
  
  void Planner2D::updateGlobalDistanceMap(const Point2fVectorCloud& scan_pts_) {
    Point2iVectorCloud scan_pts_pxl;
    pixelizeScanPoints(scan_pts_pxl, scan_pts_, *_grid_map);
    
    // bb add current laser scan to distance map
    PathMatrixDistanceSearch dmap_computator;
    dmap_computator.setPathMatrix(&_global_pmap);
    float dmax_in_pixels = param_max_point_distance.constValue() * _inverse_resolution;
    dmap_computator.param_max_distance_squared_pxl.setValue(dmax_in_pixels*dmax_in_pixels);
    dmap_computator.setGoals(scan_pts_pxl, false);
    dmap_computator.compute();
    _cost_map_changed_flag=true;
  }

}
