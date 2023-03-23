#include "controller.h"
#include <srrg_converters/converter.h>

void Controller::scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  using namespace srrg2_laser_slam_2d;
  auto base_sensor_msg = srrg2_core_ros::Converter::convert(scan);
  base_sensor_msg->topic.setValue(param_raw_data_preprocessor->param_scan_topic.value());
  srrg2_slam_interfaces::TrackerReportRecord report_record;
  report_record.clear();
  param_raw_data_preprocessor->setMeas(&_scan_points_all_normal);
  if (param_raw_data_preprocessor->setRawData(base_sensor_msg, report_record)) {
    param_raw_data_preprocessor->compute();
  }
  this->_need_redraw = true;
}

void Controller::computeTarget(const geometry_msgs::Twist& twist_input_) {
  const float linear_velocity_scaled = twist_input_.linear.x * _input_scaling;
  const float angular_velocity       = twist_input_.angular.z;
  _desired_target.x()                = linear_velocity_scaled * cos(angular_velocity);
  _desired_target.y()                = linear_velocity_scaled * sin(angular_velocity);
}

void Controller::_drawImpl(srrg2_core::ViewerCanvasPtr canvas_) const {
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  std::cerr << "drawImpl Controller!!!!" << std::endl;
  if (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    ++frame_counter;
    SystemUsageCounter::tic();
    {
      canvas_->pushPointSize();
      canvas_->setPointSize(1.5);
      canvas_->putPoints(_scan_points_all_normal);
      canvas_->popAttribute();
    }
    {
      canvas_->pushColor(); // push color
      canvas_->setColor(srrg2_core::ColorPalette::color4fGreen());
      Isometry3f desired_target_in_world              = Isometry3f::Identity();
      desired_target_in_world.translation().head<2>() = _desired_target;
      canvas_->pushMatrix();
      canvas_->multMatrix(desired_target_in_world.matrix());
      canvas_->putSphere(0.05);

      canvas_->popMatrix();
      canvas_->popAttribute(); // pop color
    }

    {
      canvas_->pushColor(); // push color
      canvas_->setColor(srrg2_core::ColorPalette::color4fMagenta());

      canvas_->putSphere(0.05);

      canvas_->popAttribute(); // pop color
    }
    _target_planner->draw(canvas_);
    canvas_->flush();
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

bool Controller::computeControl(geometry_msgs::Twist& twist, const Eigen::Vector3f& delta) {
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
  if (fabs(theta_to_point) > param_pure_rotation_threshold.value() &&
      t.norm() > param_translation_reach_threshold.value()) {
    twist.angular.z    = param_rv_gain.value() * theta_to_point;
    _status_msg.status = "initial_turning";
    return true;
  }

  if (t.norm() > param_translation_reach_threshold.value()) {
    twist.linear.x     = t.norm() * param_tv_gain.value();
    twist.angular.z    = twist.linear.x * 2 * t.y() / t.squaredNorm();
    _status_msg.status = "cruising";
    return true;
  }

  if (fabs(theta) > param_rotation_reach_threshold.value()) {
    twist.angular.z    = param_rv_gain.value() * theta;
    _status_msg.status = "finalizing";
    return true;
  }

  return false;
}

void Controller::cmdVelCallback(const geometry_msgs::Twist& twist_input) {
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
  computeTarget(twist_input);
  geometry_msgs::Twist twist_output = twist_input;
  // Vector2f voxelize_coeffs(voxelize_res, voxelize_res);
  // last_scan_points.clear();
  // scan_points_all.voxelize(std::back_insert_iterator<Point2fVectorCloud>(last_scan_points),
  //                          voxelize_coeffs);
  // toPixel(last_scan_points);
  _target_planner->reset();
  _target_planner->setDesiredTarget(_desired_target);
  _target_planner->setLaserScanPoints(_scan_points_all_normal);
  _target_planner->compute();
  if (_target_planner->status() == TargetPlanner::Status::FarFromObstacles) {
    std::cerr << "status: " << _target_planner->status() << std::endl;
    // bb No obstacles in the nearby of the desired target: parent is nullptr!
    _cmd_vel_publisher.publish(twist_input);
    _status_msg.cmd_vel_output = twist_input;
    _status_msg.status         = "clear";
    _status_publisher.publish(_status_msg);
  } else if (_target_planner->status() == TargetPlanner::Status::Error) {
    std::cerr << "status: " << _target_planner->status() << std::endl;
  } else {
    Vector3f actual_target_3f  = Vector3f::Zero();
    actual_target_3f.head<2>() = _target_planner->actualTarget();
    actual_target_3f(2)        = atan2(actual_target_3f(1), actual_target_3f(0));
    computeControl(twist_output, actual_target_3f);

    _status_msg.cmd_vel_output = twist_output;
    _cmd_vel_publisher.publish(twist_output);
    _status_publisher.publish(_status_msg);

    double tic_end = getTime();
    double toc     = tic_end - tic;
    if (toc > _max_cmd_vel_callback) {
      _max_cmd_vel_callback = toc;
    }
  }
  this->_need_redraw = true;
}
