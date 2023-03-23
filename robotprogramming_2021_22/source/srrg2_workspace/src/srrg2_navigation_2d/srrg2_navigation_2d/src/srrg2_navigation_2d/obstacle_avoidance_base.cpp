#include "obstacle_avoidance_base.h"
using namespace srrg2_core;
using namespace srrg2_navigation_2d;

#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool obs_avoidance_base_debug = false;

using namespace std;

static constexpr int ColumnOfNewImage = 500;
static constexpr int RowsOfNewImage   = 500;
static cv::Size cv_image_size(RowsOfNewImage, ColumnOfNewImage);
ObstacleAvoidanceBase::ObstacleAvoidanceBase() {
  initialize();
  _cv_image_size = cv_image_size;
}
void ObstacleAvoidanceBase::initialize() {
  _offset = Vector2f::Ones() * param_dim_local_window.value();
  DEBUG(obs_avoidance_base_debug) << "creating cost layers" << std::endl;
  int rows = _offset.y() * 2;
  int cols = _offset.x() * 2;
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
    prop_cost_dx =
      new PropertyCostType("cost_map_dx", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
  }
  prop_cost_dx->value().fill(0.f);

  PropertyCostType* prop_cost_dy = _grid_map->property<PropertyCostType>("cost_map_dy");
  if (!prop_cost_dy) {
    prop_cost_dy =
      new PropertyCostType("cost_map_dy", "", _grid_map.get(), Matrix_<float>(rows, cols), nullptr);
  }
  prop_cost_dy->value().fill(0.f);
  if (param_voxelize_res.value() == 0) {
    throw std::runtime_error("[ObstacleAvoidanceBase] voxelize_res zero!!!");
  }
  if (rows == 0 || cols == 0) {
    throw std::runtime_error("Empty distance map!!!");
  }
  _distance_map.resize(rows, cols);
  _dmap_calculator.setPathMatrix(&_distance_map);
  float max_point_distance_px = param_max_point_distance.value() / param_voxelize_res.value();
  _dmap_calculator.param_max_distance_squared_pxl.setValue(max_point_distance_px *
                                                           max_point_distance_px);
  _cmap_calculator.param_min_distance.setValue(param_robot_radius.value());
}

void ObstacleAvoidanceBase::_computeDistanceMap(const Point2iVectorCloud& pixel_point_cloud_) {
  if (!_dmap_calculator.pathMap()) {
    throw std::runtime_error("No distance map!");
  }
  _dmap_calculator.setGoals(pixel_point_cloud_);
  _dmap_calculator.compute();
}

void ObstacleAvoidanceBase::_plotDistanceMap(const PathMatrix& distance_map_) {
  // bb get srrg image from distance and parent layers
  distance_map_.toImage(_path_map_image, PathMatrix::Distance);
  distance_map_.toImage(_parent_map_image, PathMatrix::Parent);
  // bb to cv matrix
  _path_map_image.toCv(_m);
  cv::Mat colored_m;
  // bb to cv colored matrix
  cv::cvtColor(_m, colored_m, cv::COLOR_GRAY2BGR);
  _parent_map_image.toCv(_p);
  _p.convertTo(_p, CV_8U);
  _p_scaled.convertTo(_p_scaled, CV_8U);
  // bb scale both distance and parent image
  cv::resize(colored_m, _m_scaled, cv_image_size);
  cv::resize(_p, _p_scaled, cv_image_size);
  cv::imshow("distance map", _m_scaled);
  cv::imshow("parent map", _p_scaled);
}

void ObstacleAvoidanceBase::_plotDistanceMap(const PathMatrix& distance_map_,
                                             const Vector2i& actual_target_,
                                             const Vector3fVector& path_) {
  _plotDistanceMap(distance_map_);
  // bb convert cost layer to matrix
  distance_map_.toImage(_cost_map_image, PathMatrix::Cost);
  // bb convert matrix to cv matrix
  _cost_map_image.toCv(_q);
  cv::Mat colored_q;
  // bb convert cv matrix to colored cv matrix
  cv::cvtColor(_q, colored_q, cv::COLOR_GRAY2BGR);
  // bb color pixels corresponding to points in path_ green
  if (path_.size()) {
    for (const auto& element : path_) {
      Vector2i element_in_pixel = _grid_map->local2indices(element.head<2>());
      cv::Vec3b& color2 = colored_q.at<cv::Vec3b>(element_in_pixel.x(), element_in_pixel.y());
      color2            = cv::Vec3b(0, 255, 0);
    }
  }
  // bb color pixel origin blue
  Vector2i element_in_pixel = _grid_map->local2indices(Vector2f(0.f, 0.f));
  cv::Vec3b& color0         = colored_q.at<cv::Vec3b>(element_in_pixel.x(), element_in_pixel.y());
  color0                    = cv::Vec3b(255, 0, 0);
  // bb color pixel target red
  cv::Vec3b& color = colored_q.at<cv::Vec3b>(actual_target_.x(), actual_target_.y());
  color            = cv::Vec3b(0, 0, 255);
  // bb scale colored cv matrix and show it
  cv::resize(colored_q, _q_scaled, cv_image_size);
  cv::imshow("cost map", _q_scaled);
}

void ObstacleAvoidanceBase::_updateCostLayers(const PathMatrix& distance_map_) {
  Matrix_<float>& cost_map    = _grid_map->property<PropertyCostType>("cost_map")->value();
  Matrix_<float>& cost_map_dx = _grid_map->property<PropertyCostType>("cost_map_dx")->value();
  Matrix_<float>& cost_map_dy = _grid_map->property<PropertyCostType>("cost_map_dy")->value();
  cost_map.fill(-1);
  cost_map_dx.fill(0);
  cost_map_dy.fill(0);
  const size_t rows = cost_map.rows();
  const size_t cols = cost_map.cols();
  assert(rows == distance_map_.rows() && "row size mistmatch");
  assert(cols == distance_map_.cols() && "cols size mistmatch");

  // fill cost layer
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      const PathMatrixCell& cell = distance_map_.at(r, c);
      float& cost                = cost_map.at(r, c);
      if (cell.parent) {
        cost = cell.cost;
      }
    }
  }

  // bb [px] displacement for getting a more interesting landscape
  size_t displacement_for_derivation = 1;
  // fill derivatives
  for (size_t r = 1; r < rows - 1; ++r) {
    for (size_t c = 1; c < cols - 1; ++c) {
      const float& cc = cost_map.at(r, c);
      const float& cx = cost_map.at(r + displacement_for_derivation, c);
      const float& cy = cost_map.at(r, c + displacement_for_derivation);
      if (cc < 0 || cx < 0 || cy < 0) {
        continue;
      }
      cost_map_dx.at(r, c) = cx - cc;
      cost_map_dy.at(r, c) = cy - cc;
    }
  }
}

bool ObstacleAvoidanceBase::_computePolicy(const Vector2i& goal_, PathMatrix& distance_map_) {
  _cmap_calculator.setPathMatrix(&distance_map_);
  if (!distance_map_.inside(goal_)) {
    std::cerr << "goal out of map" << std::endl;
    return false;
  }
  DEBUG(obs_avoidance_base_debug) << FG_BGREEN("GOAL SET IN COMPUTE POLICY: ") << goal_.transpose()
                                  << std::endl;
  // bb set vector of goals
  Point2iVectorCloud goals;
  Point2i goal_pt;
  goal_pt.coordinates() = goal_;
  goals.emplace_back(goal_pt);
  _cmap_calculator.reset();
  // bb set cost polynomial
  _cmap_calculator.param_cost_polynomial.value().clear();
  _cmap_calculator.param_cost_polynomial.pushBack(0.05f);
  _cmap_calculator.param_cost_polynomial.pushBack(0.05f);
  int num_good_goals = _cmap_calculator.setGoals(goals);
  if (!num_good_goals) {
    std::cerr << __PRETTY_FUNCTION__ << " failed to set the goal" << std::endl;
    return false;
  }
  cerr << "cacati, distance goal: " << _distance_map.at(goal_).distance << endl;
  // bb compute the cost layer of distance_map_
  _cmap_calculator.compute();
  _updateCostLayers(distance_map_);
  // bb plot distance with patte
  _distance_map.toImage(_path_map_image, PathMatrix::Cost);
  // bb to cv matrix
  _path_map_image.toCv(_m);
  cv::Mat colored_m;
  // bb to cv colored matrix
  cv::cvtColor(_m, colored_m, cv::COLOR_GRAY2BGR);
  //  for (const auto& element : _path) {
  //    Vector2i element_in_pixel = _grid_map->local2indices(element.head<2>());
  //    cv::Vec3b& color          = colored_m.at<cv::Vec3b>(element_in_pixel.x(),
  //    element_in_pixel.y()); color                     = cv::Vec3b(0, 255, 0);
  //  }
  //  for (const auto& punto_maledetto : _targets_pxl) {
  //    cv::Vec3b& color = colored_m.at<cv::Vec3b>(punto_maledetto.x(), punto_maledetto.y());
  //    color            = cv::Vec3b(0, 0, 255);
  //  }
  //  Vector2i element_in_pixel = _grid_map->local2indices(Vector2f(0.f, 0.f));
  //  cv::Vec3b& color          = colored_m.at<cv::Vec3b>(element_in_pixel.x(),
  //  element_in_pixel.y()); color                     = cv::Vec3b(255, 0, 0); color =
  //  colored_m.at<cv::Vec3b>(desired_target_pixel.x(), desired_target_pixel.y()); color =
  //  cv::Vec3b(0, 0, 255);
  // bb scale both distance and parent image
  cv::resize(colored_m, _m_scaled, cv_image_size);
  cv::imshow("cost map", _m_scaled);
  // bb plot
  return true;
}

bool ObstacleAvoidanceBase::_computePathGradient(const srrg2_core::Vector2f& goal_) {
  if (!_grid_map) {
    return false;
  }
  _path.clear();
  // bb retrieve cost and its derivatives
  Matrix_<float>& cost_map    = _grid_map->property<PropertyCostType>("cost_map")->value();
  Matrix_<float>& cost_map_dx = _grid_map->property<PropertyCostType>("cost_map_dx")->value();
  Matrix_<float>& cost_map_dy = _grid_map->property<PropertyCostType>("cost_map_dy")->value();
  Vector2f indices_float      = _grid_map->local2floatIndices(Vector2f(0.f, 0.f));
  int max_steps               = 100000;
  int steps                   = 0;
  DEBUG(obs_avoidance_base_debug) << "start: " << Vector2f(0.f, 0.f).transpose() << std::endl;
  DEBUG(obs_avoidance_base_debug) << "start [px]: " << indices_float.transpose() << std::endl;
  DEBUG(obs_avoidance_base_debug) << "goal: " << goal_.transpose() << std::endl;
  DEBUG(obs_avoidance_base_debug) << "goal [px]: "
                                  << _grid_map->local2floatIndices(goal_).transpose() << std::endl;
  bool goal_ok = false;
  while (steps < max_steps) {
    Vector2i indices = indices_float.cast<int>();
    // outside, stop
    if (!cost_map.inside(indices)) {
      std::cerr << __PRETTY_FUNCTION__ << " goal outside cost map" << std::endl;
      return false;
    }
    // invalid stop
    if (cost_map.at(indices) < 0) {
      std::cerr << __PRETTY_FUNCTION__ << " cost map at robot position: " << cost_map.at(indices)
                << std::endl;
      std::cerr << __PRETTY_FUNCTION__ << " cost sucks" << std::endl;
      return false;
    }
    // no gradient, stop
    if (cost_map_dx.at(indices) == 0 && cost_map_dy.at(indices) == 0) {
      std::cerr << __PRETTY_FUNCTION__ << " no gradient" << std::endl;
      break;
    }
    // get interpolated gradient
    float cost_interpolated;
    Vector2f gradient;
    bool interpolation_ok = cost_map_dx.getSubPixel(gradient.x(), indices_float) &&
                            cost_map_dy.getSubPixel(gradient.y(), indices_float) &&
                            cost_map.getSubPixel(cost_interpolated, indices_float);

    // no interpolation possible, we stop
    if (!interpolation_ok) {
      std::cerr << __PRETTY_FUNCTION__ << " interpolation fail" << std::endl;
      break;
    }
    gradient.normalize();
    indices_float -= gradient;
    Vector3f trj_pose;
    trj_pose.head<2>()              = _grid_map->floatIndices2local(indices_float);
    const float& target_orientation = atan2(trj_pose.y(), trj_pose.x());
    trj_pose(2)                     = target_orientation;
    _path.push_back(trj_pose);
    ++steps;
    if ((trj_pose.head<2>() - goal_).squaredNorm() <
        param_voxelize_res.value() * param_voxelize_res.value()) {
      DEBUG(obs_avoidance_base_debug) << "difference in " << __PRETTY_FUNCTION__ << ": "
                                      << (trj_pose.head<2>() - goal_).squaredNorm() << std::endl;
      goal_ok = true;
      break;
    }
  }
  DEBUG(obs_avoidance_base_debug) << "num steps in " << __PRETTY_FUNCTION__ << ": " << steps
                                  << std::endl;
  return goal_ok;
}
