#include "ros/ros.h"
#include "navigation_2d_map_requestor_ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
#include <srrg_config/configurable_command.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

namespace srrg2_navigation_2d_ros {
  using namespace srrg2_core;
  using namespace std;

  Navigation2DMapRequestorROS::Navigation2DMapRequestorROS() {
    addCommand (new ConfigurableCommand_
                < Navigation2DMapRequestorROS,
                typeof(&Navigation2DMapRequestorROS::cmdRequestMap),
                std::string>
                (this,
                 "requestMap",
                 "requests the map to a map server and propagates it to the connected module",
                 &Navigation2DMapRequestorROS::cmdRequestMap
                 ));

  }

  bool Navigation2DMapRequestorROS::cmdRequestMap(std::string& response) {
    response= "requesting map";
    requestMap();
    return true;
  }
  
  void Navigation2DMapRequestorROS::requestMap() {
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");

    while(ros::ok() && !ros::service::call(param_static_map_service.value(), req, resp)){

      ROS_WARN_STREAM("Request for map " << param_static_map_service.value() << " failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }

    if (!ros::ok())
      ros::shutdown();
    else
      mapMessageCallback(resp.map);

  }

  void Navigation2DMapRequestorROS::mapMessageCallback(const::nav_msgs::OccupancyGrid& msg) {
    _grid_map=GridMap2DPtr(new GridMap2D);
    using PropertyImageOccupancyType=Property_<ImageOccupancyType>;
    _grid_map->setResolution(msg.info.resolution);
    PropertyImageOccupancyType* occ_prop=new PropertyImageOccupancyType("occupancy",
                                                                        "",
                                                                        _grid_map.get(),
                                                                        Matrix_<uint8_t>(),
                                                                        nullptr);
    const int rows=msg.info.width;
    const int cols=msg.info.height;
    _grid_map->setSize(Eigen::Vector2i(rows, cols));
    Matrix_<uint8_t>& occ_map=occ_prop->value();
    std::cerr << "occ_map size:" << occ_map.rows() << " " << occ_map.cols() << std::endl;
    int k=0;
    for(int c=0; c<cols; c++) {
      for(int r=0; r<rows; r++) {
	    int d=msg.data[k];
	    if (d<0) {
	      d=127; 
	    } else {
              d=(255.f/100.f)*(100-d);
              if (d==127)
                d=127;
            }
	    occ_map.at(r,c)=(unsigned char)(d);
	    k++;
      }
    }
    if (param_map_user.value()) {
      //adjust center
      Vector2f sizes=_grid_map->GridMap2DHeader::size().cast<float>()*_grid_map->resolution();
      std::cerr << "new origin"  << (sizes*.5).transpose() << std::endl;
      Eigen::Isometry2f origin=Eigen::Isometry2f::Identity();
      origin.translation()=sizes*.5;
      _grid_map->setOrigin(origin);
      param_map_user->setMap(_grid_map);
      param_map_user.value()->param_map_frame_id.setValue(msg.header.frame_id);
    }
  }

}
