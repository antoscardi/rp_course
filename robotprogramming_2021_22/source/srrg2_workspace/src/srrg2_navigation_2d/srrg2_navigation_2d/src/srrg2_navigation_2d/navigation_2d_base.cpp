#include "navigation_2d_base.h"
#include <srrg_config/configurable_command.h>
#include <srrg_data_structures/path_matrix_distance_search.h>

#include <unistd.h>
#include <iomanip>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  Navigation2DBase::Navigation2DBase() {
    addCommand(new ConfigurableCommand_<Navigation2DBase,
                                        typeof(&Navigation2DBase::cmdLoadMap),
                                        std::string,
                                        std::string>(
      this, "loadMap", "loads a map from a yaml file", &Navigation2DBase::cmdLoadMap));
  }

  bool Navigation2DBase::cmdLoadMap(std::string& response, const std::string& filename) {
    response = className() + "| loading map from  file [" + filename + "]";
    return loadMap(filename);
  }

  bool Navigation2DBase::loadMap(const std::string& filename) {
    const int line_size_max = 1024;
    ifstream is(filename.c_str());
    if (!is) {
      return false;
    }
    float resolution      = 0;
    float occupied_thresh = 0;
    float free_thresh     = 0;
    std::string image_file;
    bool negate = 0;
    while (is) {
      char line[line_size_max];
      is.getline(line, line_size_max);
      istringstream ls(line);
      std::string tag;
      ls >> tag;
      if (tag == "resolution:") {
        ls >> resolution;
        continue;
      }
      if (tag == "negate:") {
        ls >> negate;
        continue;
      }
      if (tag == "free_thresh:") {
        ls >> free_thresh;
        continue;
      }
      if (tag == "occupied_thresh:") {
        ls >> occupied_thresh;
        continue;
      }
      if (tag == "origin:") {
        continue;
      }
      if (tag == "image:") {
        ls >> image_file;
        continue;
      }
    }
    if (resolution == 0 || !image_file.length()) {
      return false;
    }
    cv::Mat cv_img = cv::imread(image_file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    ImageUInt8 image;
    image.fromCv(cv_img);
    std::cerr << "Got map" << image.rows() << "x" << image.cols() << std::endl;
    std::cerr << "res: " << resolution << std::endl;
    size_t rows = image.rows();
    size_t cols = image.cols();
    GridMap2DPtr grid_map(new GridMap2D);
    grid_map->setSize(Eigen::Vector2i(rows, cols));
    grid_map->setResolution(resolution);
    using PropertyImageOccupancyType = Property_<ImageOccupancyType>;
    param_occupancy_threshold.setValue(occupied_thresh);
    param_free_threshold.setValue(free_thresh);
    std::cerr << "free threshold: " << param_free_threshold.value() << std::endl;
    std::cerr << "occupied threshold: " << param_occupancy_threshold.value() << std::endl;

    PropertyImageOccupancyType* occ_prop = new PropertyImageOccupancyType(
      param_occupancy_layer.value(), "", grid_map.get(), Matrix_<uint8_t>(), nullptr);
    Matrix_<uint8_t>& occ_map = occ_prop->value();
    occ_map.resize(rows, cols);
    for (size_t r = 0; r < rows; ++r)
      for (size_t c = 0; c < cols; ++c) {
        occ_map.at(r, c) = image.at(r, c);
      }
    cv::Mat img2;

    // adjust external coords for ROS
    float x_size = rows * resolution;
    float y_size = cols * resolution;
    Eigen::Vector3f origin(y_size / 2, x_size / 2, -M_PI / 2);
    std::cerr << "Origin: " << origin << std::endl;
    grid_map->setOrigin(geometry2d::v2t(origin));

    setMap(grid_map);
    return true;
  }

  bool Navigation2DBase::getRobotPose(Eigen::Isometry2f& dest, double time) {
    Isometry2f robot_pose;
    bool transform_ok=platform()->getTransform(dest,
                                               param_base_link_frame_id.value(),
                                               param_map_frame_id.value(),
                                               time);
    if (! transform_ok) {
      cerr << __PRETTY_FUNCTION__ << ": could not get robot pose in map at time " << std::fixed << std::setprecision(5) << time << endl;
    }
    return transform_ok;
  }

  bool Navigation2DBase::getSensorPoseOnRobot(Eigen::Isometry2f& dest,
                                              const std::string& sensor_frame) {
    dest.setIdentity();
    if (param_tf_topic.value().length()) {
      if (!platform()) {
        std::cerr << className() << "| putMessage requires a tf tree, but not here yet, skipping"
                  << std::endl;
        return false;
      } else {
        bool tf_result =
          platform()->getTransform(dest, sensor_frame, param_base_link_frame_id.value());
        if (!tf_result) {
          std::cerr << className() << "| interpolation error in retrieving laser pose" << std::endl;
          return false;
        }
      }
    }
    return true;
  }

  // returns the endpoints in the reference frame of the robot
  // (by operating unprojection and frame transformation)
  bool Navigation2DBase::scan2endpoints(Point2fVectorCloud& dest, const LaserMessage& src) {
    Isometry2f sensor_pose_on_robot;
    if (!getSensorPoseOnRobot(sensor_pose_on_robot, src.frame_id.value()))
      return false;
    const float range_min  = std::max(src.range_min.value(), param_range_min.value());
    const float range_max  = std::min(src.range_max.value(), param_range_max.value());
    const float& angle_min = src.angle_min.value();
    const float& angle_max = src.angle_max.value();
    int num_beams          = src.ranges.size();
    int k                  = 0;
    dest.resize(num_beams);
    float alpha         = angle_min;
    const float d_alpha = (angle_max - angle_min) / num_beams;
    for (size_t i = 0; i < src.ranges.size(); ++i, alpha += d_alpha) {
      const float& range = src.ranges.value(i);
      if (range < range_min || range > range_max)
        continue;
      dest[k].status = POINT_STATUS::Valid;
      Vector2f laser_point(range * cos(alpha), range * sin(alpha));
      dest[k].coordinates() = sensor_pose_on_robot * laser_point;
      ++k;
    }
    dest.resize(k);
    return true;
  }

  void Navigation2DBase::setMap(GridMap2DPtr grid_map_) {
    std::cerr << "setting map " << std::endl;

    assert(grid_map_ && "grid map not set");
    _grid_map            = grid_map_;
    float occ_threshold  = (1. - param_occupancy_threshold.value()) * 255;
    float free_threshold = (1. - param_free_threshold.value()) * 255;
    _resolution          = _grid_map->resolution();
    _inverse_resolution  = 1. / _resolution;
    std::cerr << FG_BBLUE("GRID MAP RESOLUTION: ") << _resolution << std::endl;
    using PropertyImageOccupancyType = Property_<ImageOccupancyType>;
    PropertyImageOccupancyType* occ_prop =
      _grid_map->property<PropertyImageOccupancyType>(param_occupancy_layer.value());
    if (!occ_prop) {
      throw std::runtime_error(std::string("occupancy layer [") + param_occupancy_layer.value() +
                               "] not found in map");
    }
    _map = occ_prop->value();
    _obstacles.clear();
    const size_t rows = _map.rows();
    const size_t cols = _map.cols();

    std::cerr << "setting map " << rows << " x " << cols << std::endl;

    // DEBUG
    std::cerr << "occ_th: " << occ_threshold << " free_th:" << free_threshold << std::endl;
    int free_count = 0, occ_count = 0, unknown_count = 0;
    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        unsigned char& src = _map.at(r, c);
        if (src < occ_threshold) {
          occ_count++;
          Point2i p;
          p.coordinates() = Eigen::Vector2i(r, c);
          _obstacles.push_back(p);
          src = 0;
        } else if (src > free_threshold) {
          free_count++;
          src = 255;
        } else {
          unknown_count++;
          src = 127;
        }
      }
    }
    // std::cerr << "writing map" << std::endl;
    // cv::imwrite("proc_occ.pgm", _img_occ);
    cerr << "free: " << free_count << endl;
    cerr << "unknown: " << unknown_count << endl;
    cerr << "occupied: " << occ_count << endl;
    _distance_map.resize(rows, cols);

    PathMatrixDistanceSearch dmap_calculator;
    dmap_calculator.setPathMatrix(&_distance_map);
    float dmax_in_pixels = param_max_point_distance.value() * _inverse_resolution;
    dmap_calculator.param_max_distance_squared_pxl.setValue(dmax_in_pixels * dmax_in_pixels);
    dmap_calculator.setGoals(_obstacles);
    dmap_calculator.compute();

    Matrix_<float>* distances = nullptr;
    if (param_distance_layer.value().length()) {
      using PropertyDistanceType = Property_<Matrix_<float>>;
      PropertyDistanceType* dist_prop =
        _grid_map->property<PropertyDistanceType>(param_distance_layer.value());
      if (!dist_prop) {
        dist_prop = new PropertyDistanceType(
          param_distance_layer.value(), "float", _grid_map.get(), Matrix_<float>(), nullptr);
      }
      distances = &dist_prop->value();
      distances->resize(rows, cols);
    }

    int k = 0;
    _free_cells.resize(rows * cols);

    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        //update free list;
        unsigned char pixel     = _map.at(r, c);
        int distance_sign=(pixel==127)?-1:1;
        float &distance_px= _distance_map.at(r, c).distance;
        float &distance_mt= distances->at(r,c);
        distance_mt=distance_sign * sqrt(distance_px) * _resolution;
        distance_px*=distance_sign;
        if (pixel == 255 && distance_mt > param_robot_radius.constValue()) {
          _free_cells[k] = grid2world(r, c);
          ++k;
        }
      }
    }

    // cv::imwrite("proc_dist.pgm", _img_dist);
    _free_cells.resize(k);
    cerr << "valid: " << _free_cells.size() << endl;
    _map_changed_flag = false;
  }

  void Navigation2DBase::handleMapChanged() {
    if (!_map_changed_flag)
      return;
    setMap(this->_grid_map);
    _map_changed_flag = false;
  }
} // namespace srrg2_navigation_2d
