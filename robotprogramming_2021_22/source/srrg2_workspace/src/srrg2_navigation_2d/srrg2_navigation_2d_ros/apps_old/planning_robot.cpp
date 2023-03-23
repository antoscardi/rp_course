#include "tf/transform_listener.h"
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <srrg2_navigation_2d_msgs/PathFollowerStatus.h>
#include <srrg2_navigation_2d_ros/instances.h>
#include <srrg2_navigation_2d_ros/path_follower.h>
#include <srrg2_navigation_2d_ros/tf_helpers.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_converters/converter.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_pcl/point_types.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <thread>

using namespace srrg2_core;
using namespace srrg2_navigation_2d;
using namespace srrg2_navigation_2d_ros;
using srrg2_core_ros::Converter;

ros::Publisher cmd_vel_publisher;

std::shared_ptr<tf::TransformListener> listener;
ObstacleAvoidanceBase::Isometry2fList path_poses;
Point2fVectorCloud path_points;
GridMap2D grid_map;

// hardcoded parameters
std::string map_frame_id       = "map";
std::string cmd_vel_topic      = "/cmd_vel";
std::string path_topic         = "/path";
std::string grid_map_topic     = "grid_map";
std::string base_link_frame_id = "base_link";
std::string path_follower_name = "path_follower";

// ia nice logging :)
size_t total_path_size = 0;
const std::string exe_name("planning_robot");

#define LOG std::cerr << exe_name + "|"
#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool planning_debug = false;

void generateConfig(const std::string& config_file_) {
  ConfigurableManager manager;
  const std::string scan_topic = "/base_scan";
  auto path_follower           = manager.create<PathFollower>(path_follower_name);
  path_follower->param_scan_handler->param_raw_data_preprocessor->param_scan_topic.setValue(
    scan_topic);
  manager.write(config_file_);

  LOG << "default config saved in [ " << config_file_ << " ]\n";
}

void pathCallback(const nav_msgs::Path& path_) {
  // ia cache and log
  total_path_size = path_.poses.size();
  std::cerr << "Got path -- path size [ " << total_path_size << " ]\n";
  path_poses.clear();
  path_points.clear();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    Point2f current_point;
    Isometry2f iso;
    getIsometry(iso, path_.poses[i].pose);
    path_poses.push_back(iso);
    current_point.coordinates() = iso.translation();
    path_points.emplace_back(std::move(current_point));
  }
}

void gridMapCallback(const grid_map_msgs::GridMapConstPtr& grid_map_) {
  // bb convert to BaseSensorMessagePtr
  BaseSensorMessagePtr grid_map_msg = Converter::convert(grid_map_);
  if (!grid_map_msg) {
    std::cerr << __PRETTY_FUNCTION__ << FG_BCYAN("Was not able to convert the grid map message!!!")
              << std::endl;
    return;
  }
  // bb cast BaseSensorMessagePtr to GridMapMessagePtr
  GridMapMessagePtr grid_map_ptr = std::dynamic_pointer_cast<GridMapMessage>(grid_map_msg);
  grid_map                       = grid_map_ptr->grid_map_struct.value();
  DEBUG(planning_debug) << FG_BCYAN("GRID MAP SIZE: ") << grid_map.GridMap2DHeader::size().x()
                        << "x" << grid_map.GridMap2DHeader::size().y() << std::endl;
}

void node(ViewerCanvasPtr canvas_, std::shared_ptr<PathFollower> path_follower_) {
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("map_frame_id", map_frame_id);
  _nh.getParam("base_link_frame_id", base_link_frame_id);
  _nh.getParam("cmd_vel_topic", cmd_vel_topic);
  _nh.getParam("path_topic", path_topic);
  std::cerr << exe_name << ": running with these parameters" << std::endl;
  std::cerr << "_map_frame_id:=" << map_frame_id << std::endl;
  std::cerr << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
  std::cerr << "_cmd_vel_topic:=" << cmd_vel_topic << std::endl;
  std::cerr << "_path_topic:=" << path_topic << std::endl;

  cmd_vel_publisher                   = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
  ros::Subscriber path_subscriber     = nh.subscribe("path", 10, pathCallback);
  ros::Subscriber grid_map_subscriber = nh.subscribe(grid_map_topic, 10, gridMapCallback);
  path_follower_->setCanvas(canvas_);
  path_follower_->advertiseTopics(nh);
  ros::Subscriber scan_subscriber = nh.subscribe(
    path_follower_->param_scan_handler->param_raw_data_preprocessor->param_scan_topic.value(),
    10,
    &ScanHandler::scanCallback,
    path_follower_->param_scan_handler.value().get());

  ros::Rate loop_rate(50);
  listener.reset(new tf::TransformListener);

  // bb first of all check that the map is valid
  grid_map_msgs::GridMapConstPtr sharedPtr;
  sharedPtr = ros::topic::waitForMessage<grid_map_msgs::GridMap>(grid_map_topic, nh);
  if (sharedPtr == nullptr) {
    ROS_INFO("No grid map message received!!!!!");
  } else {
    std::cerr << FG_BMAGENTA("GRID MAP MESSAGE RECEIVED") << std::endl;
    gridMapCallback(sharedPtr);
  }
  while (ros::ok()) {
    if (!path_poses.size()) {
      DEBUG(planning_debug) << "Waiting for next path to be received!" << std::endl;
      std::flush(std::cerr);
    } else {
      Isometry2f robot_in_world;
      if (!getTfTransform(
            robot_in_world, *listener, map_frame_id, base_link_frame_id, ros::Time(0))) {
        break;
      }
      geometry_msgs::Twist twist;
      // bb set twist to zero
      // bb path_follower_ will modify it if possible
      twist.linear.x  = 0.f;
      twist.angular.z = 0.f;
      path_follower_->compute(grid_map, robot_in_world, path_points, path_poses, twist);
      // cmd_vel_publisher.publish(twist);
      // bb if goal has been reached within a given tolerance clear path_poses and path_points
      // bb get ready to receive next path
      if (path_follower_->status() == PathFollower::Status::End) {
        path_poses.clear();
        path_points.clear();
        std::cerr << "Path poses size: " << path_poses.size() << std::endl;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv) {
  static const std::string canvas_name("my_canvas");
  ros::init(argc, argv, "path_follower", ros::init_options::NoSigintHandler);
  srrg2_navigation_2d_ros_registerTypes();
  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(argc, argv, &qapp);
  const ViewerCanvasPtr& canvas = viewer_core.getCanvas(canvas_name);
  srrg2_qgl_viewport::ViewerCoreSharedQGL::stop();
  srrgInit(argc, argv, exe_name.c_str());
  ParseCommandLine cmd_line(argv);

  ArgumentFlag arg_generate_config(&cmd_line, "j", "generate-config", "generates a config file");
  ArgumentString arg_config_file(
    &cmd_line, "c", "config-file", "path to config file", "conf_" + exe_name + ".json");
  cmd_line.parse();

  const std::string& config_file = arg_config_file.value();

  // srrg check for options and params
  if (arg_generate_config.isSet()) {
    LOG << "generating default config\n";
    generateConfig(config_file);
    return 0;
  }
  ConfigurableManager config_manager;
  config_manager.read(config_file);
  std::cerr << "reading config from [" << config_file << "]" << std::endl;

  auto path_follower = config_manager.getByName<PathFollower>(path_follower_name);
  if (!path_follower) {
    throw std::runtime_error("no path follower in configuration");
  }
  auto path_planner = config_manager.getByName<LocalPathPlanner>("local_path_planner");
  if (!path_planner) {
    throw std::runtime_error("no path follower in configuration");
  }
  std::cerr << "vx res: " << path_planner->param_voxelize_res.value() << std::endl;

  node(canvas, path_follower);
  //  std::thread node_t(node, canvas, path_follower);
  //  viewer_core.startViewerServer();
  //  node_t.join();
  return 0;
}
