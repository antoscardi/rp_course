#include "target_planner.h"
#include <srrg2_laser_slam_2d/sensor_processing/raw_data_preprocessor_projective_2d.h>
#include <srrg_converters/converter.h>
#include <srrg_system_utils/system_utils.h>

static constexpr int ColumnOfNewImage = 500;
static constexpr int RowsOfNewImage   = 500;
cv::Size cv_image_size(RowsOfNewImage, ColumnOfNewImage);

void TargetPlanner::reset() {
  _gradient_dijkstra     = Vector2f::Zero();
  _gradient_distance_map = Vector2f::Zero();
  _desired_target        = Vector2f::Zero();
  _actual_target         = Vector2f::Zero();
  _path_map_image.fill(0);
  _parent_map_image.fill(0);
  _cost_map_image.fill(0);
  _distance_map.fill(PathMatrixCell());
}

Vector2f TargetPlanner::computeGradientDistanceMap(const PathMatrix& distance_map,
                                                   const Vector2i& parent_cell_,
                                                   const ThisType::Type type) {
  // bb compute gradient
  Vector2f gradient(0.f, 0.f);
  const int step_in_pixel_gradient = 10;
  const int parent_cell_x          = parent_cell_.x();
  const int parent_cell_y          = parent_cell_.y();
  auto x_east    = distance_map(parent_cell_x + step_in_pixel_gradient, parent_cell_y);
  auto x_west    = distance_map(parent_cell_x - step_in_pixel_gradient, parent_cell_y);
  auto y_south   = distance_map(parent_cell_x, parent_cell_y + step_in_pixel_gradient);
  auto y_north   = distance_map(parent_cell_x, parent_cell_y - step_in_pixel_gradient);
  float dx_east  = 0.f;
  float dx_west  = 0.f;
  float dy_south = 0.f;
  float dy_north = 0.f;
  if (type == Distance) {
    dx_east  = x_east.distance;
    dx_west  = x_west.distance;
    dy_south = y_south.distance;
    dy_north = y_north.distance;
  } else if (type == Cost) {
    dx_east  = x_east.cost;
    dx_west  = x_west.cost;
    dy_south = y_south.cost;
    dy_north = y_north.cost;
  } else {
    throw std::runtime_error("No known type for derivative computation!!!");
  }

  if (dx_east >= 0 || dx_east > 0 || dx_east > 0 || dx_east > 0) {
    gradient.x() = dx_east - dx_west;
    gradient.y() = dy_south - dy_north;
  }
  return gradient.normalized();
}

Vector2i TargetPlanner::modifyTarget(const Vector2i& target_pixel_,
                                     float squared_distance_,
                                     const Vector2f& gradient_) {
  Vector2f result = Vector2f::Zero();
  std::cout << "squared_distance: " << squared_distance_ << std::endl;
  float distance = sqrtf(squared_distance_);
  std::cout << "distance: " << distance << std::endl;
  const float distance_pixel = (param_robot_radius.value()) / param_voxelize_res.value() - distance;

  std::cout << "distance_pixel: " << distance_pixel << std::endl;
  const float scaling = distance_pixel;
  result              = target_pixel_.cast<float>() + gradient_ * scaling;
  return result.cast<int>();
}

void TargetPlanner::updateCostLayers(const PathMatrix& distance_map) const {
  Matrix_<float>& cost_map    = _grid_map->property<PropertyCostType>("cost_map")->value();
  Matrix_<float>& cost_map_dx = _grid_map->property<PropertyCostType>("cost_map_dx")->value();
  Matrix_<float>& cost_map_dy = _grid_map->property<PropertyCostType>("cost_map_dy")->value();
  cost_map.fill(-1);
  cost_map_dx.fill(0);
  cost_map_dy.fill(0);
  const size_t rows = cost_map.rows();
  const size_t cols = cost_map.cols();
  assert(rows == distance_map.rows() && "row size mistmatch");
  assert(cols == distance_map.cols() && "cols size mistmatch");

  // fill cost layer
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      const PathMatrixCell& cell = distance_map.at(r, c);
      float& cost                = cost_map.at(r, c);
      if (cell.parent) {
        cost = cell.cost;
      }
    }
  }

  // fill derivatives
  for (size_t r = 1; r < rows - 1; ++r) {
    for (size_t c = 1; c < cols - 1; ++c) {
      const float& cc = cost_map.at(r, c);
      const float& cx = cost_map.at(r + 1, c);
      const float& cy = cost_map.at(r, c + 1);
      if (cc < 0 || cx < 0 || cy < 0) {
        continue;
      }
      cost_map_dx.at(r, c) = cx - cc;
      cost_map_dy.at(r, c) = cy - cc;
    }
  }
}

bool TargetPlanner::computePolicy(const Vector2i& goal_, PathMatrix& distance_map) {
  param_cost_map_computator->setPathMatrix(&distance_map);
  if (!distance_map.inside(goal_)) {
    std::cerr << "goal out of map" << std::endl;
    return false;
  }

  std::vector<float> cost_poly;
  cost_poly.push_back(param_voxelize_res.value());
  cost_poly.push_back(10);

  param_cost_map_computator->param_cost_polynomial.setValue(cost_poly);
  Point2iVectorCloud goals;
  Point2i goal_pt;
  goal_pt.coordinates() = goal_;
  goals.push_back(goal_pt);

  param_cost_map_computator->reset();

  std::cerr << goal_.transpose() << std::endl;
  int num_good_goals = param_cost_map_computator->setGoals(goals);
  if (!num_good_goals) {
    std::cerr << "fail" << std::endl;
    return false;
  }

  param_cost_map_computator->compute();
  updateCostLayers(distance_map);
  return true;
}

bool TargetPlanner::computePathGradient(const Vector2f& current_pose) {
  if (!_grid_map) {
    return false;
  }

  _path.clear();
  Matrix_<float>& cost_map    = _grid_map->property<PropertyCostType>("cost_map")->value();
  Matrix_<float>& cost_map_dx = _grid_map->property<PropertyCostType>("cost_map_dx")->value();
  Matrix_<float>& cost_map_dy = _grid_map->property<PropertyCostType>("cost_map_dy")->value();
  Vector2f indices_float      = _grid_map->global2floatIndices(current_pose);
  int max_steps               = 100000;
  int steps                   = 0;
  std::cerr << "start: " << current_pose.transpose() << std::endl;
  std::cerr << "goal: " << _actual_target.transpose() << std::endl;
  bool goal_ok = false;
  while (steps < max_steps) {
    Vector2i indices = indices_float.cast<int>();
    // outside, stop
    if (!cost_map.inside(indices)) {
      std::cerr << "outside" << std::endl;
      return false;
    }
    // invalid stop
    if (cost_map.at(indices) < 0) {
      std::cerr << "cost map at robot position: " << cost_map.at(indices) << std::endl;
      std::cerr << "cost sucks" << std::endl;
      return false;
    }
    // no gradient, stop
    if (cost_map_dx.at(indices) == 0 && cost_map_dy.at(indices) == 0) {
      std::cerr << "no gradient" << std::endl;
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
      std::cerr << "interpolation fail" << std::endl;
      break;
    }
    indices_float -= gradient;
    Vector2f trj_pose;
    trj_pose = _grid_map->floatIndices2global(indices_float);
    _path.push_back(trj_pose);
    ++steps;
    if ((trj_pose - _actual_target).squaredNorm() <
        param_voxelize_res.value() * param_voxelize_res.value()) {
      std::cerr << "difference in computePathGradient: "
                << (trj_pose - _actual_target).squaredNorm() << std::endl;
      goal_ok = true;
      break;
    }
  }
  std::cerr << "num steps in computePathGradient: " << steps << std::endl;
  return goal_ok;
}

void TargetPlanner::computeDistanceMap(const Point2iVectorCloud& pixel_point_cloud_) {
  param_distance_map_computator->setGoals(pixel_point_cloud_);
  param_distance_map_computator->compute();
}

void TargetPlanner::plotDistanceMap() {
  _distance_map.toImage(_path_map_image, PathMatrix::Distance);
  _distance_map.toImage(_parent_map_image, PathMatrix::Parent);

  _path_map_image.toCv(_m);
  _parent_map_image.toCv(_p);
  _m.convertTo(_m, CV_8U);
  _m_scaled.convertTo(_m_scaled, CV_8U);
  _p.convertTo(_p, CV_8U);
  _p_scaled.convertTo(_p_scaled, CV_8U);

  cv::resize(_m, _m_scaled, cv_image_size);
  cv::resize(_p, _p_scaled, cv_image_size);
  cv::imshow("distance map", _m_scaled);
  cv::imshow("parent map", _p_scaled);

  for (auto current_pos : _path_indexes) {
    cv::Vec3b& color = _q_scaled.at<cv::Vec3b>(current_pos.y(), current_pos.x());
    color[2]         = 254;
  }
}

void TargetPlanner::plotDistanceMap(const PathMatrix& distance_map,
                                    const Vector2i& actual_target,
                                    const Vector2fVector& path) {
  plotDistanceMap();
  distance_map.toImage(_cost_map_image, PathMatrix::Cost);
  _cost_map_image.toCv(_q);
  cv::Mat colored_q;
  cv::cvtColor(_q, colored_q, cv::COLOR_GRAY2BGR);
  for (auto element : path) {
    Vector2i element_in_pixel = _grid_map->local2indices(element);
    cv::Vec3b& color2         = colored_q.at<cv::Vec3b>(element_in_pixel.x(), element_in_pixel.y());
    color2                    = cv::Vec3b(0, 255, 0);
  }
  cv::Vec3b& color0 = colored_q.at<cv::Vec3b>(offset.x(), offset.y());
  color0            = cv::Vec3b(255, 0, 0);
  cv::Vec3b& color  = colored_q.at<cv::Vec3b>(actual_target.x(), actual_target.y());
  color             = cv::Vec3b(0, 0, 255);

  cv::resize(colored_q, _q_scaled, cv_image_size);
  cv::imshow("cost map", _q_scaled);
}

void TargetPlanner::_drawImpl(srrg2_core::ViewerCanvasPtr canvas) const {
  if (_status != Status::Success) {
    return;
  }
  std::string text       = "testing viewer manager";
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  std::cerr << "drawImpl TargetPlanner!!!!" << std::endl;
  if (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    ++frame_counter;
    SystemUsageCounter::tic();
    {
      const float length      = 0.4 /*_gradient_dijkstra.norm()*/;
      const float head_width  = 0.1;
      const float head_length = 0.1;
      Vector3f arrow_properties(length, head_width, head_length);
      canvas->pushColor(); // push color
      canvas->setColor(srrg2_core::ColorPalette::color4fGreen());
      Isometry3f gradient_dijkstra_isometry = Isometry3f::Identity();
      const float rotation                  = atan2(_gradient_dijkstra.y(), _gradient_dijkstra.x());
      Matrix3f rotation_matrix;
      rotation_matrix << cos(rotation), -sin(rotation), 0, sin(rotation), cos(rotation), 0, 0, 0, 1;
      gradient_dijkstra_isometry.linear() = rotation_matrix;
      canvas->pushMatrix();
      canvas->multMatrix(gradient_dijkstra_isometry.matrix());
      canvas->putArrow2D(arrow_properties);

      canvas->popMatrix();
      canvas->popAttribute(); // pop color
    }
    {
      const float length      = _gradient_distance_map.norm(); /*_gradient_dijkstra.norm()*/
      const float head_width  = 0.1;
      const float head_length = 0.1;
      Vector3f arrow_properties(length, head_width, head_length);
      canvas->pushColor(); // push color
      canvas->setColor(srrg2_core::ColorPalette::color4fEerieBlack());
      Isometry3f gradient_distance_isometry = Isometry3f::Identity();
      const float rotation = atan2(_gradient_distance_map.y(), _gradient_distance_map.x());
      Matrix3f rotation_matrix;
      rotation_matrix << cos(rotation), -sin(rotation), 0, sin(rotation), cos(rotation), 0, 0, 0, 1;
      gradient_distance_isometry.linear() = rotation_matrix;
      canvas->pushMatrix();
      canvas->multMatrix(gradient_distance_isometry.matrix());
      canvas->putArrow2D(arrow_properties);

      canvas->popMatrix();
      canvas->popAttribute(); // pop color
    }
    {
      canvas->pushColor(); // push color
      canvas->setColor(srrg2_core::ColorPalette::color3fViolet());
      Isometry3f desired_target_parent_in_world              = Isometry3f::Identity();
      desired_target_parent_in_world.translation().head<2>() = _desired_target_parent;
      canvas->pushMatrix();
      canvas->multMatrix(desired_target_parent_in_world.matrix());
      canvas->putSphere(0.05);

      canvas->popMatrix();
      canvas->popAttribute(); // pop color
    }
    {
      canvas->pushColor(); // push color
      canvas->setColor(srrg2_core::ColorPalette::color4fOrange());
      Isometry3f actual_target_in_world              = Isometry3f::Identity();
      actual_target_in_world.translation().head<2>() = _actual_target_draw;
      canvas->pushMatrix();
      canvas->multMatrix(actual_target_in_world.matrix());
      canvas->putSphere(0.05);

      canvas->popMatrix();
      canvas->popAttribute(); // pop color
    }
    // canvas->flush();
    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
  }
}

void TargetPlanner::compute() {
  // bb needed despite the use of the grid map as the grid map only represents the local map
  Point2iVectorCloud pixel_point_cloud;
  pixel_point_cloud.resize(_laser_scan_points.size());
  Point2i point_indices;
  for (auto point : _laser_scan_points) {
    srrg2_core::GridMap2D::IndicesType indices = _grid_map->local2indices(point.coordinates());
    point_indices.coordinates()                = indices;
    pixel_point_cloud.emplace_back(point_indices);
  }
  computeDistanceMap(pixel_point_cloud);
  PathMatrix distance_map_dijkstra = _distance_map;
  // bb use the above to skip the distances and only consider the distance to the goal
  auto desired_target_pixel  = _grid_map->local2indices(_desired_target);
  auto desired_target_parent = _distance_map(desired_target_pixel).parent;
  // only for debug
  _desired_target_parent = _grid_map->indices2local(_distance_map.pos(desired_target_parent));

  float desired_target_pixel_distance = _distance_map(desired_target_pixel).distance;
  std::cout << "desired target pixel distance: " << desired_target_pixel_distance << std::endl;

  float robot_radius_pixel = param_robot_radius.value() / param_voxelize_res.value();
  if (desired_target_pixel_distance > robot_radius_pixel * robot_radius_pixel) {
    // bb No obstacles near the desired target
    _status = ThisType::Status::FarFromObstacles;
    std::cerr << "Far from obstacles!!!" << std::endl;
    return;
  }
  Vector2f distance_map_gradient =
    computeGradientDistanceMap(_distance_map, desired_target_pixel, ThisType::Type::Distance);
  Vector2i actual_target_pixel =
    modifyTarget(desired_target_pixel, desired_target_pixel_distance, distance_map_gradient);
  _actual_target         = _grid_map->indices2local(actual_target_pixel);
  _actual_target_draw    = _actual_target;
  _gradient_distance_map = distance_map_gradient;
  bool dijkstra_done     = computePolicy(actual_target_pixel, distance_map_dijkstra);
  if (!dijkstra_done) {
    _status = ThisType::Status::Error;
    std::cerr << "status: " << _status << std::endl;
    throw std::runtime_error("Could not do Dijkstra!");
  }
  Vector2f gradient =
    computeGradientDistanceMap(distance_map_dijkstra, offset.cast<int>(), ThisType::Type::Cost);
  _gradient_dijkstra = -gradient;
  std::cerr << "gradient!!!!!!!!!!" << gradient << std::endl;
  bool gradient_ok = computePathGradient(Vector2f::Zero());
  if (!gradient_ok) {
    std::cerr << "no path found to goal" << std::endl;
    std::cerr << "status: " << _status << std::endl;
    _status = ThisType::Status::Error;
    return;
  }
  std::cerr << "gradient path ok, size:" << _path.size() << std::endl;
  _path_indexes.clear();
  Vector2f difference = _actual_target - _path[0];
  std::cerr << "actual target - path[0]: " << difference.squaredNorm() << std::endl;
  _actual_target = _path[0];
  plotDistanceMap(distance_map_dijkstra, actual_target_pixel, _path);
  _status            = ThisType::Status::Success;
  this->_need_redraw = true;
}
