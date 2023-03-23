#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <srrg2_navigation_2d/navigation_2d_base.h>
#include <srrg_config/property_configurable.h>
namespace srrg2_navigation_2d_ros {
  using namespace srrg2_core;
  using namespace srrg2_navigation_2d;

  class Navigation2DMapRequestorROS : public srrg2_core::Configurable {
  public:
    PARAM(PropertyString, map_topic, "name of the map topic", "/map", nullptr);

    PARAM(PropertyString,
          static_map_service,
          "name of the service producing the static map",
          "static_map",
          nullptr);

    PARAM(PropertyConfigurable_<Navigation2DBase>,
          map_user,
          "module to which the map will be injected",
          nullptr,
          nullptr);
    Navigation2DMapRequestorROS();
    void requestMap();
    void mapMessageCallback(const nav_msgs::OccupancyGrid& msg);
    bool cmdRequestMap(std::string& response);
    GridMap2DPtr _grid_map;
  };

} // namespace srrg2_navigation_2d_ros
