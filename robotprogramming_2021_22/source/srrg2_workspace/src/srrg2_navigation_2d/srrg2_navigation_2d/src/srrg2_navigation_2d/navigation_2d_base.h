#pragma once
//#include "srrg_types/defs.h"
#include <srrg_config/configurable.h>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_data_structures/path_matrix.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_pcl/point.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;

  using Vector2fVector = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  class Navigation2DBase : public MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(PropertyFloat, range_min, "range_min [meters]", 0.0, nullptr);

    PARAM(PropertyFloat, range_max, "range_max [meters]", 1000.0, nullptr);

    PARAM(PropertyFloat,
          max_point_distance,
          "max expansion distance for the dmap",
          3.0f,
          &this->_map_changed_flag);

    PARAM(PropertyFloat, robot_radius, "radius of the robot [m]", 0.2f, &this->_map_changed_flag);

    PARAM(PropertyString,
          occupancy_layer,
          "name of the layer containing the occupancy values",
          "occupancy",
          &this->_map_changed_flag);

    PARAM(PropertyString,
          distance_layer,
          "name of the layer containing the distance values",
          "distances",
          &this->_map_changed_flag);

    PARAM(PropertyString, map_frame_id, "frame id of the map", "/map", nullptr);

    PARAM(PropertyFloat,
          occupancy_threshold,
          "values below this will be regarded as occupied 0 is black",
          0.51,
          &this->_map_changed_flag);

    PARAM(PropertyFloat,
          free_threshold,
          "values above this will be regarded as occupied 1 is white",
          0.49,
          &this->_map_changed_flag);

    PARAM(PropertyString, base_link_frame_id, "frame id of the robot origin", "base_link", nullptr);

    Navigation2DBase();

    //! call this to load the map in the localizer
    //! @param m: an 8 bit grayscale image
    //! @param resolution: size in meters of a map pixel
    //! @param occ_threshold: values of a pixel values below this are considered occupies
    //! @param free_threshold: values of a pixel above this are free space
    virtual void setMap(GridMap2DPtr grid_map);

    bool loadMap(const std::string& filename);

    // shell commands
    bool cmdLoadMap(std::string& response, const std::string& filename);

  protected:
    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) const {
      assert(_grid_map && "grid map not set");
      return _grid_map->global2indices(p);
    }

    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) const {
      assert(_grid_map && "grid map not set");
      return _grid_map->indices2global(p);
    }

    inline Eigen::Vector2i world2grid(float x, float y) const {
      assert(_grid_map && "grid map not set");
      return _grid_map->global2indices(Vector2f(x, y));
    }

    inline Eigen::Vector2f grid2world(int r, int c) const {
      assert(_grid_map && "grid map not set");
      return _grid_map->indices2global(Vector2i(r, c));
    }

    virtual void handleMapChanged();
    // returns the endpoints in the reference frame of the robot
    // (by operating unprojection and frame transformation)
    bool scan2endpoints(Point2fVectorCloud& dest, const LaserMessage& src);
    bool getSensorPoseOnRobot(Eigen::Isometry2f& dest, const std::string& sensor_frame);
    bool getRobotPose(Eigen::Isometry2f& dest, double time);

    Vector2fVector _free_cells;
    srrg2_core::Matrix_<uint8_t> _map;
    srrg2_core::GridMap2DPtr _grid_map = nullptr;
    float _resolution, _inverse_resolution;
    mutable srrg2_core::PathMatrix _distance_map;
    Matrix_<float> _distance_matrix;
    srrg2_core::Point2iVectorCloud _obstacles;
    bool _map_changed_flag = true;
  };

} // namespace srrg2_navigation_2d
