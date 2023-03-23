#include "instances.h"
//#include "motion_controller.h"
#include "navigation_2d_map_requestor_ros.h"
//#include "path_follower.h"
//#include "scan_handler.h"
//#include <srrg2_laser_slam_2d/instances.h>
#include <srrg2_navigation_2d/instances.h>
#include <srrg_pcl/instances.h>

namespace srrg2_navigation_2d_ros {

  void srrg2_navigation_2d_ros_registerTypes() {
    //    srrg2_laser_slam_2d::srrg2_laser_slam_2d_registerTypes();
    //srrg2_core::point_cloud_registerTypes();
    srrg2_navigation_2d::srrg2_navigation_2d_registerTypes();
    BOSS_REGISTER_CLASS(Navigation2DMapRequestorROS)
      //BOSS_REGISTER_CLASS(PathFollower)
      //BOSS_REGISTER_CLASS(ScanHandler)
      //BOSS_REGISTER_CLASS(MotionController)
  }

} // namespace srrg2_navigation_2d_ros
