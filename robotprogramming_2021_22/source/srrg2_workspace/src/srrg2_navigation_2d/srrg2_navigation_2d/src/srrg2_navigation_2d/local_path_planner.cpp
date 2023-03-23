#include "local_path_planner.h"
#include <opencv2/core/eigen.hpp>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_system_utils/system_utils.h>
using namespace srrg2_navigation_2d;
using namespace srrg2_core;

static constexpr int ColumnOfNewImage = 500;
static constexpr int RowsOfNewImage   = 500;
static cv::Size cv_image_size(RowsOfNewImage, ColumnOfNewImage);

#define LOG std::cerr << exe_name + "|"

LocalPathPlanner::LocalPathPlanner() {
  initialize();
}

void LocalPathPlanner::setOrigin(const srrg2_core::Isometry2f& origin_) {
  // bb store origin so you can use it in compute to set the origin again
  _origin = origin_;
  _grid_map->setOrigin(origin_);
}

void LocalPathPlanner::setGlobalGridMap(const srrg2_core::GridMap2D& global_grid_map_) {
  DEBUG(local_path_planner_debug) << FG_BCYAN("GRID MAP SIZE: ")
                                  << _global_grid_map.GridMap2DHeader::size().x() << "x"
                                  << _global_grid_map.GridMap2DHeader::size().y() << std::endl;
  _global_grid_map = global_grid_map_;
  std::cerr << __PRETTY_FUNCTION__ << std::endl;
  DEBUG(local_path_planner_debug) << FG_BCYAN("GRID MAP SIZE: ")
                                  << _global_grid_map.GridMap2DHeader::size().x() << "x"
                                  << _global_grid_map.GridMap2DHeader::size().y() << std::endl;
}

void LocalPathPlanner::_publishPath() {
  static int count = 0;
  std::cerr << "In publishPath!!!!" << std::endl;
  PathMessagePtr path(new PathMessage);
  path->topic.setValue("/local_path");
  path->frame_id.setValue("/base_link");
  path->timestamp.setValue(getTime());
  path->seq.setValue(++count);

  for (size_t i = 0; i < _path.size(); ++i) {
    PoseStampedMessage pose;
    pose.topic.setValue("/local_path");
    pose.frame_id.setValue("/base_link");
    pose.timestamp.setValue(getTime());
    pose.seq.setValue(i);
    pose.pose.value().setPose(_path[i]);
    path->poses.pushBack(pose);
  }
  propagateMessage(path);
}

bool LocalPathPlanner::putMessage(BaseSensorMessagePtr msg_) {
  // handle(msg)
  if (_path_changed_flag) {
    _publishPath();
    _path_changed_flag = false;
    return true;
  }
  return false;
}

void LocalPathPlanner::compute() {
  srrg2_core::Chrono chrono_time("totalChrono: ", &_chrono_map, true);
  if (_distance_map_changed_flag) {
    initialize();
    _distance_map_changed_flag = false;
  }
  // bb set origin of local grid map
  _grid_map->setOrigin(_origin);
  // bb needed despite the use of the grid map as the grid map only represents the local map
  Point2iVectorCloud pixel_point_cloud;
  pixel_point_cloud.resize(_scan_points.size());
  for (const auto& point : _scan_points) {
    srrg2_core::GridMap2D::IndicesType indices = _grid_map->local2indices(point.coordinates());
    Point2i point_indices;
    point_indices.coordinates() = indices;
    pixel_point_cloud.push_back(std::move(point_indices));
  }
  {
    srrg2_core::Chrono chrono_time("distanceMapChrono: ", &_chrono_map, true);
    _computeDistanceMap(pixel_point_cloud);
  }
  _setDistanceMapForDijkstra();
  Vector2i desired_target_pixel = _grid_map->local2indices(_desired_target);
  // bb plot distance with patte
  _distance_map.toImage(_path_map_image, PathMatrix::Distance);
  // bb to cv matrix
  _path_map_image.toCv(_m);
  cv::Mat colored_m;
  // bb to cv colored matrix
  cv::cvtColor(_m, colored_m, cv::COLOR_GRAY2BGR);
  Vector2i element_in_pixel = _grid_map->local2indices(Vector2f(0.f, 0.f));
  cv::Vec3b& color          = colored_m.at<cv::Vec3b>(element_in_pixel.x(), element_in_pixel.y());
  color                     = cv::Vec3b(255, 0, 0);
  cv::Vec3b& color0 = colored_m.at<cv::Vec3b>(desired_target_pixel.x(), desired_target_pixel.y());
  color0            = cv::Vec3b(0, 0, 255);
  // bb scale both distance and parent image
  cv::resize(colored_m, _m_scaled, cv_image_size);
  cv::imshow("distance map", _m_scaled);
  // PathMatrix distance_map_dijkstra = _distance_map;

  // bb trying to avoid a copy of distance_map
  DEBUG(local_path_planner_debug) << "desired target pixel: " << desired_target_pixel.transpose()
                                  << std::endl;
  DEBUG(local_path_planner_debug) << "desired target: " << _desired_target.transpose() << std::endl;
  bool dijkstra_done = false;
  {
    srrg2_core::Chrono chrono_time("costMapChrono: ", &_chrono_map, true);
    dijkstra_done = ObstacleAvoidanceBase::_computePolicy(desired_target_pixel, _distance_map);
  }
  if (!dijkstra_done) {
    // bb return empty local path
    _path.clear();
    DEBUG(local_path_planner_debug) << FG_BGREEN("Dijkstra not doable!!!") << std::endl;
    DEBUG(local_path_planner_debug)
      << __PRETTY_FUNCTION__ << " returning an empty path!" << std::endl;
  }
  bool gradient_ok = _computePathGradient(_desired_target);
  if (!gradient_ok) {
    std::cerr << FG_BMAGENTA("no path found to goal") << std::endl;
    _status = ThisType::Status::Error;
    return;
  }
  DEBUG(local_path_planner_debug) << "gradient path ok, size:" << _path.size() << std::endl;
  _status = ThisType::Status::Success;
  // bb plot distance, parent and cost map
  // bb paint in cost map origin blue, path green and target red
  //_plotDistanceMap(_distance_map, desired_target_pixel, _path);

  // publish path
  _path_changed_flag = true;
  this->_need_redraw = true;
}

void LocalPathPlanner::_setDistanceMapForDijkstra() {
  using IndicesType = Vector2i;
  // bb to be able to perform Dijkstra which assumes distances expressed in meters
  // bb take the square root of each field and multiply by the resolution
  // bb take the minimum between the local and the global distance
  float local_distance  = 0.f;
  float global_distance = 0.f;
  // bb retrieve distance layer from global grid map
  const Matrix_<float>& global_distance_map =
    _global_grid_map.property<PropertyCostType>("distances")->value();
  ImageFloat img;
  StdVectorEigenVector2i idx;
  Vector2i origin_in_pxl   = _global_grid_map.global2indices(geometry2d::t2v(_origin).head<2>());
  Vector2i desired_tgt_pxl = _global_grid_map.global2indices(_desired_target_in_world);
  idx.emplace_back(origin_in_pxl.y(), origin_in_pxl.x());
  idx.emplace_back(desired_tgt_pxl.y(), desired_tgt_pxl.x());
  size_t col_size = global_distance_map.cols() / 4;
  size_t row_size = global_distance_map.rows() / 4;
  img.matrixWithPoints2cv("merda", global_distance_map, idx, col_size, row_size);
  // bb for debugging a matrix of global_distances and a matrix of local_distances and compare them
  // bb further you can draw global_indices on global_distance_map using matrixWithPoints2cv
  // bb when using matrix2cv that is a method of Image, use e.g. ImageFloat, remember that rows and
  // cols are inverted
  for (size_t row = 0; row < _distance_map.rows(); ++row) {
    for (size_t col = 0; col < _distance_map.cols(); ++col) {
      IndicesType local_indices(row, col);
      IndicesType global_indices(row, col);
      PathMatrixCell& cell = _distance_map.at(local_indices);
      local_distance       = sqrtf(cell.distance) * param_voxelize_res.value();
      global_indices  = _global_grid_map.global2indices(_grid_map->indices2global(local_indices));
      global_distance = global_distance_map.at(global_indices);
      cell.distance   = global_distance; /*std::min(local_distance, global_distance);*/
    }
  }
  std::cerr << "local" << local_distance << std::endl;
  std::cerr << "global" << global_distance << std::endl;
}

bool LocalPathPlanner::setPath(const srrg2_core::Point2fVectorCloud& points_,
                               srrg2_core::StdVectorEigenVector2f& targets_) {
  if (points_.empty()) {
    std::cerr << "[LocalPathPlanner] Empty path!" << std::endl;
    return false;
  }
  //_path_points_in_robot = points_;
  size_t i                         = 0;
  size_t points_size               = points_.size();
  srrg2_core::Vector2i next_target = srrg2_core::Vector2i::Zero();
  srrg2_core::StdVectorEigenVector2i targets;
  while (i < points_size) {
    next_target = _grid_map->global2indices(points_[i].coordinates());
    if (_distance_map.inside(next_target)) {
      break;
    }
    ++i;
  }
  while (i < points_size) {
    next_target = _grid_map->global2indices(points_[i].coordinates());
    if (!_distance_map.inside(next_target)) {
      break;
    }
    targets.emplace_back(next_target);
    targets_.emplace_back(points_[i].coordinates());
    ++i;
  }
  size_t size_target_vector = targets.size();
  // bb empty target vector
  if (!size_target_vector) {
    return false;
  }
  size_t idx_desired = (size_target_vector > 2) ? size_target_vector - 3 : size_target_vector - 1;
  _targets_pxl.clear();
  _targets_pxl.resize(targets.size());
  _targets_pxl = targets;
  setDesiredTarget(targets[idx_desired]);
  _desired_target_in_world = targets_[idx_desired];
  return true;
}

bool LocalPathPlanner::_computePolicy(const Point2iVectorCloud& goals_,
                                      srrg2_core::PathMatrix& distance_map_) {
  _cmap_calculator.setPathMatrix(&distance_map_);

  _cost_poly.push_back(param_voxelize_res.value());
  _cost_poly.push_back(10);

  _cmap_calculator.param_cost_polynomial.setValue(_cost_poly);

  _cmap_calculator.reset();
  int num_good_goals = _cmap_calculator.setGoals(goals_);
  if (!num_good_goals) {
    std::cerr << "fail" << std::endl;
    return false;
  }

  _cmap_calculator.compute();
  _updateCostLayers(distance_map_);
  return true;
}
