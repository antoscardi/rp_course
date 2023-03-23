#include "localizer_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_config/configurable_command.h>
#include <srrg_data_structures/path_matrix_dijkstra_search.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_messages/messages/pose_array_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point.h>
#include <unistd.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  Particle::Particle() {
    _pose.setZero();
    _weight = 1.0f;
  }

  Localizer2D::Localizer2D() {
    addCommand(
      new ConfigurableCommand_<Localizer2D, typeof(&Localizer2D::cmdStartGlobal), std::string>(
        this, "startGlobal", "starts the global locallization", &Localizer2D::cmdStartGlobal));
    addCommand(new ConfigurableCommand_<Localizer2D,
                                        typeof(&Localizer2D::cmdSetPose),
                                        std::string,
                                        float,
                                        float,
                                        float>(
      this, "setPose", "sets the pose x y theta", &Localizer2D::cmdSetPose));

    param_noise_coeffs.value() << 0.01, 0.0005, 0.0002, 0.0005, 0.0001, 0.0001, 0.001, 0.00001,
      0.05;
  }

  Eigen::Vector3f Localizer2D::sampleFromFreeSpace() {
    int r       = drand48() * (_free_cells.size() - 1);
    float theta = (drand48() - 0.5) * 2 * M_PI;
    return Eigen::Vector3f(_free_cells[r].x(), _free_cells[r].y(), theta);
  }

  bool Localizer2D::startGlobal() {
    if (!_grid_map)
      return false;
    _particles.resize(param_num_particles.value());
    for (size_t i = 0; i < _particles.size(); i++) {
      _particles[i]._pose   = sampleFromFreeSpace();
      _particles[i]._weight = 1;
    }
    param_particle_resetting.setValue(true);
    return true;
  }

  bool Localizer2D::cmdStartGlobal(std::string& response) {
    response = "starting global localization with " + std::to_string(param_num_particles.value()) +
               " particles";
    return Localizer2D::startGlobal();
  }

  bool Localizer2D::cmdSetPose(std::string& response, float x, float y, float theta) {
    response = "";
    setPose(Eigen::Vector3f(x, y, theta));
    return true;
  }

  bool Localizer2D::setPose(const Eigen::Vector3f pose, Eigen::Vector3f standard_deviations) {
    if (!_grid_map)
      return false;
    _particles.resize(param_num_particles.value());
    Eigen::Isometry2f pose_transform = geometry2d::v2t(pose);
    for (size_t i = 0; i < _particles.size(); i++) {
      Eigen::Vector3f noise;
      for (int k = 0; k < 3; k++)
        noise[k] = standard_deviations[k] * _normal_generator(_random_generator);
      _particles[i]._pose   = geometry2d::t2v(pose_transform * geometry2d::v2t(noise));
      _particles[i]._weight = 1;
    }
    _force_update = true;
    param_particle_resetting.setValue(false);
    return true;
  }

  void Localizer2D::prepareSampling(const Eigen::Vector3f& control) {
    Eigen::Vector3f scales(fabs(control[0]), fabs(control[1]), fabs(control[2]));
    _std_deviations = param_noise_coeffs.value() * scales;
    for (size_t i = 0; i < 3; i++) {
      _std_deviations[i] = sqrt(_std_deviations[i]);
    }
    _last_control = control;
  }

  Eigen::Vector3f Localizer2D::sample(const Eigen::Vector3f& old_state) {
    Eigen::Vector3f noise;
    for (int i = 0; i < 3; i++)
      noise[i] = _std_deviations[i] * _normal_generator(_random_generator);

    noise += _last_control;
    Eigen::Isometry2f transform = geometry2d::v2t(old_state) * geometry2d::v2t(noise);
    return geometry2d::t2v(transform);
  }

  void Localizer2D::predict(const Eigen::Vector3f control) {
    if (control.squaredNorm() < 1e-6)
      return;
    // std::cerr << className() <<  "| predict. control: " << control.transpose() << std::endl;
    prepareSampling(control);
    for (size_t i = 0; i < _particles.size(); i++)
      _particles[i]._pose = sample(_particles[i]._pose);
    _cum_translation += control.head<2>().norm();
    _cum_rotation += fabs(control[2]);
  }

  //! uniform resampling algorithm
  //! indices: the indices of the particles that survive after resampling
  //! weights: the weights of the particles as input
  //! n: the number of particles
  void resampleUniform(int* indices, const double* weights, int n) {
    double acc      = 0;
    const double* w = weights;
    for (int i = 0; i < n; i++, w++) {
      acc += *w;
    }
    double inverse_acc = 1. / acc;
    double cum_value   = 0;
    double step        = 1. / n;
    double threshold   = step * drand48();
    int* idx           = indices;

    w     = weights;
    int k = 0;
    for (int i = 0; i < n; i++, w++) {
      cum_value += (*w) * inverse_acc;
      while (cum_value > threshold) {
        *idx = i;
        idx++;
        k++;
        threshold += step;
      }
    }
  }

  double Localizer2D::likelihood(const Eigen::Vector3f& pose,
                                 bool compute_endpoint_distances) const {
    Eigen::Isometry2f iso = _grid_map->origin().inverse() * geometry2d::v2t(pose);

    Matrix_<float>* distances = nullptr;
    if (param_distance_layer.value().length()) {
      using PropertyDistanceType = Property_<Matrix_<float>>;
      PropertyDistanceType* dist_prop =
        _grid_map->property<PropertyDistanceType>(param_distance_layer.value());
      if (!dist_prop) {
        throw std::runtime_error("no dmap in localizer");
      }
      distances = &dist_prop->value();
    }

    // handle the robot out of the map
    Eigen::Vector2i p = _grid_map->local2indices(iso.translation());
    if (!distances->inside(p))
      return 0;
    // handle the robot in the unknown
    float d = distances->at(p);

    // robot in the wall, likelihood 0
    if (d < param_robot_radius.value())
      return 0;

    if (compute_endpoint_distances) {
      _endpoint_distances.resize(_last_scan_points.size());
      std::fill(_endpoint_distances.begin(), _endpoint_distances.end(), -1);
    }
    // handle the beams
    float cum_distance = 0;
    int valid_points   = 0;
    for (size_t i = 0; i < _last_scan_points.size(); ++i) {
      p = _grid_map->local2indices(iso * _last_scan_points[i].coordinates());
      if (!distances->inside(p))
        continue;
      float distance = fabs(distances->at(p));
      cum_distance += distance;
      valid_points++;
      if (compute_endpoint_distances)
        _endpoint_distances[i] = distance;
    }
    // if too less beams are good, ignore
    if (valid_points < param_min_likelihood_points.value())
      return param_min_weight.value();

    // heuristic but effective likelihood
    cum_distance /= valid_points;
    cum_distance *= param_likelihood_gain.value();
    return exp(-cum_distance) + param_min_weight.value();
  }

  bool Localizer2D::update(double timestamp) {
    // refresh the last endpoints
    _best_likelihood              = 0;
    _worst_likelihood             = std::numeric_limits<double>::max();
    double time_since_last_update = timestamp - _last_update_time;
    bool do_update = _force_update || _cum_rotation > param_min_update_rotation.value() ||
                     _cum_translation > param_min_update_translation.value();

    if (param_force_update_time.value() > 0) {
      do_update |= time_since_last_update > param_force_update_time.value();
    }

    if (!do_update) {
      return 0;
    }
    _last_update_time = timestamp;

    _force_update = false;
    // update
    _cum_translation = 0;
    _cum_rotation    = 0;
    _cum_likelihood  = 0;

    for (size_t i = 0; i < _particles.size(); i++) {
      // compute the weight of each particle
      float w = likelihood(_particles[i]._pose);
      // if the weight is 0 and replace the particle with a random one,
      // otherwise assign a weight to a particle, based on the likelihood
      if (w == 0 && param_particle_resetting.value()) {
        w                   = param_min_weight.value();
        _particles[i]._pose = sampleFromFreeSpace();
      }
      _particles[i]._weight = w;
      _cum_likelihood += w;
      _best_likelihood  = std::max(_best_likelihood, (double) w);
      _worst_likelihood = std::min(_worst_likelihood, (double) w);
    }
    if (_cum_likelihood < 0)
      return false;
    _indices.resize(_particles.size());
    _weights.resize(_particles.size());
    // resample
    int* indices    = &_indices[0];
    double* weights = &_weights[0];
    for (size_t i = 0; i < _particles.size(); i++)
      weights[i] = _particles[i]._weight;

    resampleUniform(indices, weights, _particles.size());
    if (_cum_likelihood == 0) {
      return true;
    }

    ParticleVector aux(_particles.size());
    int* idx = indices;
    for (size_t i = 0; i < _particles.size(); i++) {
      aux[i]         = _particles[*idx];
      aux[i]._weight = 1;
      idx++;
    }
    _particles = aux;
    return true;
  }

  void Localizer2D::computeStats() {
    Eigen::Vector2f translational_mean;
    ;
    Eigen::Vector2f angular_mean;
    translational_mean.setZero();
    angular_mean.setZero();
    // computes the mean. To calculate the angular component sum
    // the vectors (cos(theta), sin(theta) and recover the global
    // orientation
    for (size_t i = 0; i < _particles.size(); i++) {
      translational_mean += _particles[i]._pose.head<2>();
      float theta = _particles[i]._pose[2];
      angular_mean += Eigen::Vector2f(cos(theta), sin(theta));
    }
    _mean.head<2>() = translational_mean * (1. / _particles.size());
    _mean[2]        = atan2(angular_mean.y(), angular_mean.x());

    _covariance.setZero();
    for (size_t i = 0; i < _particles.size(); i++) {
      Eigen::Vector3f dp = _particles[i]._pose - _mean;
      dp[2]              = fmod(dp[2], 2 * M_PI);
      if (dp[2] > M_PI)
        dp[2] -= M_PI;
      _covariance += dp * dp.transpose();
    }
    _covariance *= 1. / _particles.size();

    _endpoint_distances.resize(_last_scan_points.size());
    double l = likelihood(_mean, true);
    if (l < param_min_weight.value()) {
      std::fill(_endpoint_distances.begin(), _endpoint_distances.end(), -1);
    }
    // std::cerr << "mean: " <<_mean.transpose()
    //           << " ml: " << l
    //           << " wl: " << _worst_likelihood
    //           << " bl: " << _best_likelihood
    //           << " cl: " << _cum_likelihood << std::endl;
  }

  bool Localizer2D::handleSetPose(BaseSensorMessagePtr msg_) {
    if (msg_->topic.value() != param_initial_pose_topic.value())
      return false;
    PoseWithCovarianceStampedMessagePtr set_pose_msg =
      std::dynamic_pointer_cast<PoseWithCovarianceStampedMessage>(msg_);
    if (!set_pose_msg) {
      std::cerr << "cast error in setting pose" << std::endl;
      return false;
    }
    std::cerr << "setting pose" << std::endl;
    Isometry2f iso =
      geometry3d::get2dFrom3dPose(geometry3d::v2t(set_pose_msg->pose.value().pose_vector.value()));
    setPose(geometry2d::t2v(iso));
    _force_update = true;
    return true;
  }

  bool Localizer2D::handleOdom(BaseSensorMessagePtr msg_) {
    if (msg_->topic.value() != param_odom_topic.value())
      return false;

    OdometryMessagePtr odom_msg = std::dynamic_pointer_cast<OdometryMessage>(msg_);
    if (!odom_msg) {
      std::cerr << className() << " ptr:" << this << " cast error on odom mesage" << std::endl;
      return false;
    }
    _odom_frame_id                 = odom_msg->frame_id.value();
    Eigen::Isometry2f current_odom = geometry3d::get2dFrom3dPose(odom_msg->pose.value());
    if (!_first_frame) {
      Eigen::Isometry2f delta_transform = _previous_odom.inverse() * current_odom;
      Eigen::Vector3f delta_vector      = geometry2d::t2v(delta_transform);
      predict(delta_vector);
    } else {
      _scan_topics.clear();
      // instantiate the projectors
      for (const std::string& laser_topic : param_scan_topics.value()) {
        _scan_topics.insert(laser_topic);
      }
    }
    _previous_odom = current_odom;
    _first_frame   = false;
    return true;
  }

  bool Localizer2D::handleScan(BaseSensorMessagePtr msg_) {
    LaserMessagePtr scan = std::dynamic_pointer_cast<LaserMessage>(msg_);
    if (!scan) {
      return false;
    }
    if (!_scan_topics.count(scan->topic.value()))
      return false;

    Point2fVectorCloud points;
    if (!Navigation2DBase::scan2endpoints(points, *scan))
      return false;

    float vox_res = std::max(_grid_map->resolution(), param_scan_voxelize_resolution.value());
    _last_scan_points.clear();
    _last_scan_points.reserve(scan->ranges.size());
    Eigen::Vector2f res_coeffs(vox_res, vox_res);
    points.voxelize(std::back_insert_iterator<Point2fVectorCloud>(_last_scan_points), res_coeffs);
    _last_publish_time = scan->timestamp.value();
    _is_updated        = update(scan->timestamp.value());
    return true;
    ;
  }

  void Localizer2D::publishParticles() {
    if (!param_publish_particles.value() || !_is_updated || !param_push_sinks.size() ||
        _particles.empty())
      return;
    PoseArrayMessagePtr pose_array_msg(new PoseArrayMessage);
    pose_array_msg->seq.setValue(++_num_updates);
    pose_array_msg->timestamp.setValue(_last_publish_time);
    pose_array_msg->topic.setValue("particles");
    pose_array_msg->frame_id.setValue(param_map_frame_id.value());
    pose_array_msg->poses.resize(_particles.size());
    for (size_t i = 0; i < _particles.size(); ++i) {
      pose_array_msg->poses.value(i).setPose(_particles[i]._pose);
    }
    propagateMessage(pose_array_msg);
  }

  void Localizer2D::publishTransform() {
    if (!platform())
      return;
    Isometry3f local_odom_3;
    bool tf_result = platform()->getTransform(
      local_odom_3, param_base_link_frame_id.value(), _odom_frame_id, _last_publish_time);
    if (!tf_result) {
      std::cerr << endl;
      std::cerr << platform() << std::endl;
      return;
    }

    Isometry2f local_odom = geometry3d::get2dFrom3dPose(local_odom_3);
    Isometry2f map_pose   = geometry2d::v2t(mean());
    TransformEventsMessagePtr transform_event(new TransformEventsMessage);
    transform_event->seq.setValue(_num_updates);
    transform_event->timestamp.setValue(_last_update_time);
    transform_event->frame_id.setValue(param_map_frame_id.value());
    transform_event->topic.setValue(param_tf_topic.value());
    transform_event->events.resize(1);
    Isometry3f dp = geometry3d::get3dFrom2dPose(map_pose * local_odom.inverse());
    TransformEvent ev(_last_publish_time, _odom_frame_id, dp, param_map_frame_id.value());
    transform_event->events.setValue(0, ev);
    propagateMessage(transform_event);
  }

  bool Localizer2D::putMessage(BaseSensorMessagePtr msg_) {
    // at the first odometry
    //    e instantiate the projector map, but we don't know the scan parameters yet
    // at each subsequent odometry,
    //    do predict
    // at the first scan on a topic
    //    we instantiate the projector in the map (once)
    // at each scan (in the scan topics) we
    //    unproject the points,
    //    voxelize them according to the map res
    //    dp update

    if (!_grid_map) {
      std::cerr << className() << " ptr:" << this << " waiting for map" << std::endl;
      return false;
    }
    _is_updated = false;
    handleMapChanged();
    if (handleSetPose(msg_)) {
      return true;
    }

    if (handleOdom(msg_)) {
      return true;
    }

    if (!handleScan(msg_)) {
      return false;
    }
    computeStats();
    publishTransform();

    if (_is_updated) {
      publishParticles();
    }

    return true;
  }

} // namespace srrg2_navigation_2d
