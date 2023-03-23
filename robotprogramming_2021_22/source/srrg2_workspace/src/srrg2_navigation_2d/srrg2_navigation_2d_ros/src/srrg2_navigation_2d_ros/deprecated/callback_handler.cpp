#include "callback_handler.h"
#include <srrg2_laser_slam_2d/sensor_processing/raw_data_preprocessor_projective_2d.h>
#include <srrg_converters/converter.h>
#include <srrg_system_utils/system_utils.h>

Vector2f CallbackHandler::offset(100, 100);

Point2iVectorCloud CallbackHandler::toPixel(const PointNormal2fVectorCloud& points_) {
  Point2iVectorCloud pixel_vector_cloud;
  pixel_vector_cloud.resize(points_.size());
  Point2i point;
  for (size_t i = 0; i < points_.size(); i++) {
    float current_point_x   = std::floor(points_[i].coordinates().x() / voxelize_res);
    float current_point_y   = std::floor(points_[i].coordinates().y() / voxelize_res);
    point.coordinates().x() = (int) -current_point_y + offset.y();
    point.coordinates().y() = (int) current_point_x + offset.x();
    pixel_vector_cloud[i]   = point;
  }
  return pixel_vector_cloud;
}

Vector2i CallbackHandler::toPixel(const Vector2f& point_) {
  PointNormal2fVectorCloud point_cloud;
  point_cloud.resize(1);
  point_cloud[0].coordinates() = point_;
  Point2iVectorCloud point_cloud_pixel;
  point_cloud_pixel = toPixel(point_cloud);
  return point_cloud_pixel[0].coordinates();
}

Vector2f CallbackHandler::fromPixel(const Vector2i& pixel_) {
  Vector2f point;
  point.x() = (pixel_.y() - offset.y()) * voxelize_res;
  point.y() = -(pixel_.x() - offset.x()) * voxelize_res;
  return point;
}

PointNormal2fVectorCloud CallbackHandler::computeNormal(const Point2fVectorCloud& point_cloud_) {
  if (point_cloud_.size() == 0) {
    std::string msg = "Point cloud is empty!";
    throw std::runtime_error(msg);
  }
  PointNormal2fVectorCloud point_normal_cloud;
  NormalComputator1DSlidingWindowNormal normal_computator;
  normal_computator.param_max_curvature.setValue(0.9);
  PointNormal2fVectorCloud current_vector_cloud;
  float voxelize_res = 0.8;
  float squared_res  = voxelize_res * voxelize_res;
  Vector2f difference(0.f, 0.f);
  PointNormal2f normal_point;
  normal_point.coordinates() = point_cloud_[0].coordinates();
  current_vector_cloud.emplace_back(normal_point);
  for (size_t i = 1; i < point_cloud_.size(); ++i) {
    difference = point_cloud_[i].coordinates() - point_cloud_[i - 1].coordinates();
    if (difference.squaredNorm() < squared_res) {
      // LOG(point_cloud_[i].coordinates());
      normal_point.coordinates() = point_cloud_[i].coordinates();
      current_vector_cloud.emplace_back(normal_point);
    } else {
      normal_computator.computeNormals(current_vector_cloud);
      point_normal_cloud.insert(
        point_normal_cloud.end(), current_vector_cloud.begin(), current_vector_cloud.end());
      current_vector_cloud.clear();
    }
  }
  normal_computator.computeNormals(current_vector_cloud);
  point_normal_cloud.insert(
    point_normal_cloud.end(), current_vector_cloud.begin(), current_vector_cloud.end());
  current_vector_cloud.clear();
  return point_normal_cloud;
}

Vector2f CallbackHandler::computeGradientDistanceMap(PathMatrixCell* parent_cell_) {
  // bb compute gradient
  Vector2f gradient(0.f, 0.f);
  Vector2i pos_in_pixel;
  const int* neighbor_offsets = _distance_map.eightNeighborOffsets();
  PathMatrixCell* down        = parent_cell_ + neighbor_offsets[1];
  pos_in_pixel                = _distance_map.pos(down);
  std::cerr << "down indexes: " << pos_in_pixel << std::endl;
  PathMatrixCell* up = parent_cell_ + neighbor_offsets[6];
  pos_in_pixel       = _distance_map.pos(up);
  std::cerr << "up indexes: " << pos_in_pixel << std::endl;
  PathMatrixCell* left = parent_cell_ + neighbor_offsets[3];
  pos_in_pixel         = _distance_map.pos(left);
  std::cerr << "left indexes: " << pos_in_pixel << std::endl;
  PathMatrixCell* right = parent_cell_ + neighbor_offsets[4];
  pos_in_pixel          = _distance_map.pos(right);
  std::cerr << "right indexes: " << pos_in_pixel << std::endl;
  gradient.x() = right->distance - left->distance;
  gradient.y() = down->distance - up->distance;
  // Vector2i parent_pos_in_pixel = _distance_map.pos(parent_cell_);
  // const float& dd              = _distance_map(parent_pos_in_pixel).distance;
  // const float& dx = _distance_map(parent_pos_in_pixel.x() + 1, parent_pos_in_pixel.y()).distance;
  // const float& dy = _distance_map(parent_pos_in_pixel.x(), parent_pos_in_pixel.y() + 1).distance;
  // if (dd >= 0 || dx >= 0 || dy >= 0) {
  //   gradient.x() = dx - dd;
  //   gradient.y() = dy - dd;
  // }
  return gradient.normalized();
}

Vector2f CallbackHandler::computeGradientDistanceMap(const Vector2i& parent_cell_) {
  // bb compute gradient
  Vector2f gradient(0.f, 0.f);
  const int parent_cell_x = parent_cell_.x();
  const int parent_cell_y = parent_cell_.y();
  const float& dx_east    = _distance_map(parent_cell_x + 10, parent_cell_y).distance;
  const float& dx_west    = _distance_map(parent_cell_x - 10, parent_cell_y).distance;
  const float& dy_south   = _distance_map(parent_cell_x, parent_cell_y + 10).distance;
  const float& dy_north   = _distance_map(parent_cell_x, parent_cell_y - 10).distance;
  if (dx_east >= 0 || dx_east > 0 || dx_east > 0 || dx_east > 0) {
    gradient.x() = dx_east - dx_west;
    gradient.y() = dy_south - dy_north;
  }
  return gradient.normalized();
}

// void CallbackHandler::scanCallback(const sensor_msgs::LaserScan& scan) {
//   double tic = getTime();
//   if (!scan2points(scan))
//     return;
//   last_scan_stamp = scan.header.stamp.toSec();
//   double toc      = getTime() - tic;
//   std::cerr << "scanCallback execution time: " << toc << std::endl;
// }

void CallbackHandler::scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  using namespace srrg2_laser_slam_2d;
  auto base_sensor_msg = srrg2_core_ros::Converter::convert(scan);
  base_sensor_msg->topic.setValue(param_raw_data_preprocessor->param_scan_topic.value());
  srrg2_slam_interfaces::TrackerReportRecord report_record;
  report_record.clear();
  param_raw_data_preprocessor->setMeas(&_scan_points_all_normal);
  if (param_raw_data_preprocessor->setRawData(base_sensor_msg, report_record)) {
    param_raw_data_preprocessor->compute();
  }
}

bool CallbackHandler::scan2points(const sensor_msgs::LaserScan& scan) {
  Isometry2f laser_in_robot;
  if (!getTfTransform(
        laser_in_robot, *_listener, base_link_frame_id, scan.header.frame_id, ros::Time(0))) {
    return false;
  }
  float crop_range = scan.range_max;
  // static Point2fVectorCloud scan_points_all;
  Point2fVectorCloud scan_points_all;
  scan_points_all.reserve(scan.ranges.size());
  scan_points_all.clear();
  float alpha = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i, alpha += scan.angle_increment) {
    float r = scan.ranges[i];
    if (r < scan.range_min)
      continue;
    if (r > crop_range)
      continue;
    float c = cos(alpha), s = sin(alpha);
    Point2f p;
    p.coordinates().x() = r * c;
    p.coordinates().y() = r * s;
    scan_points_all.push_back(p);
  }
  scan_points_all.transformInPlace(laser_in_robot);
  _scan_points_all_normal = computeNormal(scan_points_all);
  return true;
}

void CallbackHandler::computeTarget(const geometry_msgs::Twist& twist_input_) {
  const float linear_velocity_scaled = twist_input_.linear.x * input_scaling;
  const float angular_velocity       = twist_input_.angular.z;
  _desired_target.x()                = linear_velocity_scaled * cos(angular_velocity);
  _desired_target.y()                = linear_velocity_scaled * sin(angular_velocity);
  _actual_target                     = _desired_target;
}

void CallbackHandler::modifyTarget(const PointNormal2f& parent_in_world_, float distance_) {
  Vector2f normal = parent_in_world_.normal();
  const float scaling =
    /*robot_radius - std::sqrt(distance_) * voxelize_res*/ 1 / std::sqrt(distance_) * voxelize_res;
  std::cerr << "robot_radius: " << robot_radius << std::endl;
  std::cerr << "distance: " << distance_ << std::endl;
  std::cerr << "voxelize_res: " << voxelize_res << std::endl;

  if (scaling < 0.f) {
    MatrixXd distance_map_values;
    distance_map_values.resize(2 * offset.x(), 2 * offset.y());
    for (int rows = 0; rows < distance_map_values.rows(); ++rows) {
      for (int cols = 0; cols < distance_map_values.cols(); ++cols) {
        Vector2i pos;
        pos << rows, cols;
        distance_map_values(rows, cols) = _distance_map(pos).distance;
      }
    }
    std::cerr << "distance_map_values" << distance_map_values << std::endl;
    throw std::runtime_error("Desired target closer than robot_radius or did not understand the "
                             "conversion between meters and pixel!!!");
  }
  _actual_target = /*parent_in_world_.coordinates()*/
    _desired_target + normal * 2.f * scaling;
}

Vector2i CallbackHandler::modifyTarget(Vector2i& target_pixel_,
                                       float squared_distance_,
                                       const Vector2f& gradient_) {
  Vector2f result;
  const float robot_radius_pixel = robot_radius / voxelize_res;
  const float scaling            = robot_radius_pixel * 1.5;
  std::cerr << "robot_radius: " << robot_radius << std::endl;
  std::cerr << "distance: " << squared_distance_ << std::endl;
  std::cerr << "voxelize_res: " << voxelize_res << std::endl;
  result = target_pixel_.cast<float>() + gradient_ * scaling;
  return result.cast<int>();
}

void CallbackHandler::updateCostLayers() const {
  Matrix_<float>& cost_map    = _grid_map->property<PropertyCostType>("cost_map")->value();
  Matrix_<float>& cost_map_dx = _grid_map->property<PropertyCostType>("cost_map_dx")->value();
  Matrix_<float>& cost_map_dy = _grid_map->property<PropertyCostType>("cost_map_dy")->value();
  cost_map.fill(-1);
  cost_map_dx.fill(0);
  cost_map_dy.fill(0);
  const size_t rows = cost_map.rows();
  const size_t cols = cost_map.cols();
  assert(rows == _distance_map.rows() && "row size mistmatch");
  assert(cols == _distance_map.cols() && "cols size mistmatch");

  // fill cost layer
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      const PathMatrixCell& cell = _distance_map.at(r, c);
      float& cost                = cost_map.at(r, c);
      if (cell.parent)
        cost = cell.cost;
    }
  }

  // fill derivatives
  for (size_t r = 1; r < rows - 1; ++r) {
    for (size_t c = 1; c < cols - 1; ++c) {
      const float& cc = cost_map.at(r, c);
      const float& cx = cost_map.at(r + 1, c);
      const float& cy = cost_map.at(r, c + 1);
      if (cc < 0 || cx < 0 || cy < 0)
        continue;
      cost_map_dx.at(r, c) = cx - cc;
      cost_map_dy.at(r, c) = cy - cc;
    }
  }
}

bool CallbackHandler::computePolicy(const Vector2i& goal_) {
  if (!_distance_map.inside(goal_)) {
    std::cerr << "goal out of map" << std::endl;
    return false;
  }

  _path_search.setPathMatrix(&_distance_map);
  std::vector<float> cost_poly;
  float robot_radius_pixel = robot_radius / voxelize_res;
  _path_search.param_min_distance.setValue(robot_radius_pixel * robot_radius_pixel);
  cost_poly.push_back(voxelize_res);
  cost_poly.push_back(10);

  _path_search.param_cost_polynomial.setValue(cost_poly);
  Point2iVectorCloud goals;
  Point2i goal_pt;
  goal_pt.coordinates() = goal_;
  goals.push_back(goal_pt);

  _path_search.reset();

  std::cerr << goal_.transpose() << std::endl;
  int num_good_goals = _path_search.setGoals(goals);
  if (!num_good_goals) {
    std::cerr << "fail" << std::endl;
    return false;
  }

  _path_search.compute();
  // updateCostLayers();
  return true;
}

void CallbackHandler::draw_in_cmd_vel_callback() {
  std::string text = "testing viewer manager";
  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  if (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    ++frame_counter;
    SystemUsageCounter::tic();
    // ia draw a colored points polygon{}
    {
      _canvas->pushPointSize();
      _canvas->setPointSize(1.5);
      _canvas->putPoints(_scan_points_all_normal);
      _canvas->popAttribute();
    }
    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fGreen());
      Isometry3f desired_target_in_world              = Isometry3f::Identity();
      desired_target_in_world.translation().head<2>() = _desired_target;
      _canvas->pushMatrix();
      _canvas->multMatrix(desired_target_in_world.matrix());
      _canvas->putSphere(0.05);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }

    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fMagenta());

      _canvas->putSphere(0.05);

      _canvas->popAttribute(); // pop color
    }
    {
      const float length      = 0.4 /*_gradient_dijkstra.norm()*/;
      const float head_width  = 0.1;
      const float head_length = 0.1;
      Vector3f arrow_properties(length, head_width, head_length);
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fGreen());
      Isometry3f gradient_dijkstra_isometry = Isometry3f::Identity();
      const float rotation                  = atan2(_gradient_dijkstra.y(), _gradient_dijkstra.x());
      Matrix3f rotation_matrix;
      rotation_matrix << cos(rotation), -sin(rotation), 0, sin(rotation), cos(rotation), 0, 0, 0, 1;
      gradient_dijkstra_isometry.linear() = rotation_matrix;
      _canvas->pushMatrix();
      _canvas->multMatrix(gradient_dijkstra_isometry.matrix());
      _canvas->putArrow2D(arrow_properties);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }
    {
      const float length      = _gradient_distance_map.norm(); /*_gradient_dijkstra.norm()*/
      const float head_width  = 0.1;
      const float head_length = 0.1;
      Vector3f arrow_properties(length, head_width, head_length);
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fEerieBlack());
      Isometry3f gradient_distance_isometry = Isometry3f::Identity();
      const float rotation = atan2(_gradient_distance_map.y(), _gradient_distance_map.x());
      Matrix3f rotation_matrix;
      rotation_matrix << cos(rotation), -sin(rotation), 0, sin(rotation), cos(rotation), 0, 0, 0, 1;
      gradient_distance_isometry.linear() = rotation_matrix;
      _canvas->pushMatrix();
      _canvas->multMatrix(gradient_distance_isometry.matrix());
      _canvas->putArrow2D(arrow_properties);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }
    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color3fViolet());
      Isometry3f desired_target_parent_in_world = Isometry3f::Identity();
      desired_target_parent_in_world.translation().head<2>() =
        _desired_target_parent_with_normal.coordinates();
      _canvas->pushMatrix();
      _canvas->multMatrix(desired_target_parent_in_world.matrix());
      _canvas->putSphere(0.05);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }
    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fOrange());
      Isometry3f actual_target_in_world              = Isometry3f::Identity();
      actual_target_in_world.translation().head<2>() = _actual_target;
      _canvas->pushMatrix();
      _canvas->multMatrix(actual_target_in_world.matrix());
      _canvas->putSphere(0.05);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }
    _canvas->flush();
    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
  }
}

void CallbackHandler::draw() {
  std::string text = "testing viewer manager";
  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  if (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    ++frame_counter;
    SystemUsageCounter::tic();
    // ia draw a colored points polygon{}
    {
      _canvas->pushPointSize();
      _canvas->setPointSize(1.5);
      _canvas->putPoints(_scan_points_all_normal);
      _canvas->popAttribute();
    }
    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fGreen());
      Isometry3f desired_target_in_world              = Isometry3f::Identity();
      desired_target_in_world.translation().head<2>() = _desired_target;
      _canvas->pushMatrix();
      _canvas->multMatrix(desired_target_in_world.matrix());
      _canvas->putSphere(0.05);

      _canvas->popMatrix();
      _canvas->popAttribute(); // pop color
    }

    {
      _canvas->pushColor(); // push color
      _canvas->setColor(srrg2_core::ColorPalette::color4fMagenta());

      _canvas->putSphere(0.05);

      _canvas->popAttribute(); // pop color
    }
    _canvas->flush();
    duration += SystemUsageCounter::toc();

    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
    //    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
  }
}

void CallbackHandler::computeDistanceMap(Point2iVectorCloud pixel_point_cloud_) {
  _dmap_calculator.setGoals(pixel_point_cloud_);
  _dmap_calculator.compute();
}

void CallbackHandler::plotDistanceMap() {
  _distance_map.toImage(_path_map_image, PathMatrix::Distance);
  _distance_map.toImage(_parent_map_image, PathMatrix::Parent);
  _distance_map.toImage(_cost_map_image, PathMatrix::Cost);
  _path_map_image.toCv(_m);
  _parent_map_image.toCv(_p);
  _cost_map_image.toCv(_q);
  _m.convertTo(_m, CV_8U);
  _m_scaled.convertTo(_m_scaled, CV_8U);
  _p.convertTo(_p, CV_8U);
  _p_scaled.convertTo(_p_scaled, CV_8U);
  _q.convertTo(_q, CV_8UC3);
  _q_scaled.convertTo(_q_scaled, CV_8U);
  int ColumnOfNewImage = 500;
  int RowsOfNewImage   = 500;
  cv::resize(_m, _m_scaled, cv::Size(ColumnOfNewImage, RowsOfNewImage));
  cv::resize(_p, _p_scaled, cv::Size(ColumnOfNewImage, RowsOfNewImage));
  cv::resize(_q, _q_scaled, cv::Size(ColumnOfNewImage, RowsOfNewImage));
  cv::imshow("distance map", _m_scaled);
  cv::imshow("parent map", _p_scaled);

  for (auto current_pos : _path_indexes) {
    cv::Vec3b& color = _q_scaled.at<cv::Vec3b>(current_pos.y(), current_pos.x());
    color[2]         = 254;
  }

  cv::imshow("cost map", _q_scaled);
}

PointNormal2f CallbackHandler::pixelToWorld(PathMatrixCell* desired_target_parent_,
                                            const Point2iVectorCloud pixel_point_cloud_) {
  Vector2i parent_in_pixel = _distance_map.pos(desired_target_parent_);

  auto it = std::find_if(pixel_point_cloud_.begin(),
                         pixel_point_cloud_.end(),
                         [parent_in_pixel](const auto& point_in_pixel) {
                           bool x_coordinates_equal =
                             point_in_pixel.coordinates().x() == parent_in_pixel.x();
                           bool y_coordinates_equal =
                             point_in_pixel.coordinates().y() == parent_in_pixel.y();
                           return x_coordinates_equal && y_coordinates_equal;
                         });
  if (it == pixel_point_cloud_.end()) {
    throw std::runtime_error("Parent not found when expected!!!");
  }
  auto ptr_diff                 = std::distance(pixel_point_cloud_.begin(), it);
  auto ptr                      = _scan_points_all_normal.begin() + ptr_diff;
  PointNormal2f parent_in_world = *ptr;
  return parent_in_world;
}

bool CallbackHandler::computeControl(geometry_msgs::Twist& twist, const Eigen::Vector3f& delta) {
  twist.linear.x  = 0;
  twist.linear.y  = 0;
  twist.linear.z  = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  // std::cerr << "c";
  Eigen::Vector2f t = delta.head<2>();
  float theta       = delta.z();
  // goal reached condition
  // handle goal behind the target with pure rotation
  float theta_to_point = atan2(t.y(), t.x());
  if (fabs(theta_to_point) > pure_rotation_threshold && t.norm() > translation_reach_threshold) {
    twist.angular.z    = rv_gain * theta_to_point;
    _status_msg.status = "initial_turning";
    return true;
  }

  if (t.norm() > translation_reach_threshold) {
    twist.linear.x     = t.norm() * tv_gain;
    twist.angular.z    = twist.linear.x * 2 * t.y() / t.squaredNorm();
    _status_msg.status = "cruising";
    return true;
  }

  if (fabs(theta) > rotation_reach_threshold) {
    twist.angular.z    = rv_gain * theta;
    _status_msg.status = "finalizing";
    return true;
  }

  return false;
}

Vector2f CallbackHandler::computeGradientDijkstra() {
  Vector2f gradient(0.f, 0.f);
  const int parent_cell_x = offset.x();
  const int parent_cell_y = offset.y();
  const float& dx_east    = _distance_map(parent_cell_x + 10, parent_cell_y).cost;
  const float& dx_west    = _distance_map(parent_cell_x - 10, parent_cell_y).cost;
  const float& dy_south   = _distance_map(parent_cell_x, parent_cell_y + 10).cost;
  const float& dy_north   = _distance_map(parent_cell_x, parent_cell_y - 10).cost;
  if (dx_east >= 0 || dx_east > 0 || dx_east > 0 || dx_east > 0) {
    gradient.x() = dx_east - dx_west;
    gradient.y() = dy_south - dy_north;
  }
  return gradient.normalized();
}

Vector3f CallbackHandler::applyGradient(const Vector2f& gradient_) {
  Vector3f next_point(0.f, 0.f, 0.f);
  next_point.head<2>() = gradient_;
  next_point(2)        = atan2(next_point.y(), next_point.x());
  return next_point;
}

Vector2i CallbackHandler::applyGradient(const Vector2f& gradient_, bool b) {
  Vector2f robot_in_pixel = offset.cast<float>();
  Vector2f result         = gradient_ + robot_in_pixel;
  return result.cast<int>();
}

void CallbackHandler::cmdVelCallback(const geometry_msgs::Twist& twist_input) {
  double tic = getTime();

  _status_msg.header.stamp  = ros::Time::now();
  _status_msg.cmd_vel_input = twist_input;
  if (!_scan_points_all_normal.size()) {
    _cmd_vel_publisher.publish(twist_input);
    _status_msg.cmd_vel_output = twist_input;
    _status_msg.status         = "clear";
    _status_publisher.publish(_status_msg);
    return;
  }
  // compute the twist variation

  // Vector2f voxelize_coeffs(voxelize_res, voxelize_res);
  // last_scan_points.clear();
  // scan_points_all.voxelize(std::back_insert_iterator<Point2fVectorCloud>(last_scan_points),
  //                          voxelize_coeffs);
  // toPixel(last_scan_points);
  Point2iVectorCloud pixel_point_cloud = toPixel(_scan_points_all_normal);
  computeDistanceMap(pixel_point_cloud);
  computeTarget(twist_input);
  auto desired_target_pixel           = toPixel(_desired_target);
  auto desired_target_parent          = _distance_map(desired_target_pixel).parent;
  float desired_target_pixel_distance = _distance_map(desired_target_pixel).distance;
  // auto robot_cell_parent            = _distance_map(offset.y(), offset.x()).parent;
  geometry_msgs::Twist twist_output = twist_input;
  float robot_radius_pixel          = robot_radius / voxelize_res;
  if (desired_target_pixel_distance > robot_radius_pixel * robot_radius_pixel) {
    // bb No obstacles in the nearby of the desired target: parent is nullptr!
    _cmd_vel_publisher.publish(twist_input);
    _status_msg.cmd_vel_output = twist_input;
    _status_msg.status         = "clear";
    _status_publisher.publish(_status_msg);
    return;
  }
  // Vector2f gradient        = computeNextDirection(_distance_map, robot_cell_parent);

  PointNormal2f parent_in_world      = pixelToWorld(desired_target_parent, pixel_point_cloud);
  _desired_target_parent_with_normal = parent_in_world;
  // Vector2f distance_map_gradient     = computeGradientDistanceMap(desired_target_parent);
  Vector2f distance_map_gradient = computeGradientDistanceMap(desired_target_pixel);
  Vector2i actual_target_pixel =
    modifyTarget(desired_target_pixel, desired_target_pixel_distance, distance_map_gradient);
  // parent_in_world.normal() << distance_map_gradient.x(), distance_map_gradient.y();
  // _gradient_distance_map = parent_in_world.normal();
  // modifyTarget(parent_in_world, desired_target_pixel_distance);
  // updateVelocity(twist_output, parent_in_world);
  // applyNextDirection(twist_output, gradient);
  // bb due to change of coordinates between points in the distance map and points in the robot
  // frame we have to the following coordinate transformation for the vector gradient:
  _gradient_distance_map << distance_map_gradient.y(), -distance_map_gradient.x();
  // auto actual_target_pixel = toPixel(_actual_target);
  bool dijkstra_done = computePolicy(actual_target_pixel);
  if (!dijkstra_done) {
    throw std::runtime_error("Could not do Dijkstra!");
  }
  PathMatrixCell* current_parent = nullptr;
  Vector2i current_pos;
  current_pos = offset.cast<int>();
  _path_indexes.emplace_back(current_pos);
  current_parent = _distance_map(current_pos).parent;
  // PathMatrixCell* last_parent = _distance_map(actual_target_pixel).parent;
  while (current_pos != actual_target_pixel) {
    current_parent = _distance_map(current_pos).parent;
    current_pos    = _distance_map.pos(current_parent);
    _path_indexes.emplace_back(current_pos);
  }
  plotDistanceMap();
  _path_indexes.clear();
  _actual_target = fromPixel(actual_target_pixel);
  Vector2f gradient;
  gradient = -computeGradientDijkstra();
  // gradient << -gradient.x(), gradient.y();
  // gradient            = _actual_target.normalized();
  _gradient_dijkstra << gradient.y(), -gradient.x();
  // Vector3f next_point = applyGradient(gradient);
  Vector2i next_point_pixel = applyGradient(gradient, true);
  Vector3f next_point;
  next_point.head<2>() = fromPixel(next_point_pixel);
  next_point(2)        = atan2(next_point.y(), next_point.x());
  computeControl(twist_output, next_point);

  _status_msg.cmd_vel_output = twist_output;
  _cmd_vel_publisher.publish(twist_output);
  _status_publisher.publish(_status_msg);

  double tic_end = getTime();
  double toc     = tic_end - tic;
  if (toc > _max_cmd_vel_callback) {
    _max_cmd_vel_callback = toc;
  }

  draw_in_cmd_vel_callback();
}
