#include <srrg2_navigation_2d_ros/controller.h>
#include <stdexcept>
#include <thread>

using namespace srrg2_core;

void node(ViewerCanvasPtr canvas_, char** argv_) {
  std::string cmd_vel_input_topic  = "/cmd_vel_input";
  std::string cmd_vel_output_topic = "/cmd_vel";
  std::string scan_topic           = "/base_scan";
  std::string status_topic         = "/collision_avoider_status";
  ros::Publisher cmd_vel_publisher;
  ros::Publisher status_publisher;
  srrg2_navigation_2d_msgs::CollisionAvoiderStatus status_msg;
  while (!srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ros::NodeHandle nh;
  // nh.getParam("cmd_vel_input_topic", cmd_vel_input_topic);
  // nh.getParam("cmd_vel_output_topic", cmd_vel_output_topic);
  // nh.getParam("scan_topic", scan_topic);
  // nh.getParam("base_link_frame_id", base_link_frame_id);
  // nh.getParam("robot_radius", robot_radius);
  // std::cerr << "robot_radius" << robot_radius << std::endl;
  // nh.getParam("voxelize_res", voxelize_res);
  // nh.getParam("angular_loss", angular_loss);
  // nh.getParam("max_angular_correction", max_angular_correction);
  // nh.getParam("verbose", verbose);
  std::cerr << argv_[0] << ": running with params" << std::endl;

  std::cerr << "_cmd_vel_input_topic:=" << cmd_vel_input_topic << std::endl;
  std::cerr << "_cmd_vel_output_topic:=" << cmd_vel_output_topic << std::endl;
  std::cerr << "_scan_topic:=" << scan_topic << std::endl;
  std::cerr << "_status_topic:=" << status_topic << std::endl;

  status_publisher =
    nh.advertise<srrg2_navigation_2d_msgs::CollisionAvoiderStatus>(status_topic, 10);
  cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(cmd_vel_output_topic, 10);
  Controller controller(status_msg, cmd_vel_publisher, status_publisher, scan_topic);
  ros::Subscriber cmd_vel_input_subscriber =
    nh.subscribe(cmd_vel_input_topic, 10, &Controller::cmdVelCallback, &controller);
  ros::Subscriber scan_subscriber =
    nh.subscribe(scan_topic, 10, &Controller::scanCallback, &controller);
  ros::Rate loop_rate(50);

  cv::namedWindow("distance map");
  cv::namedWindow("parent map");
  while (ros::ok()) {
    ros::spinOnce();
    controller.draw(canvas_);
    loop_rate.sleep();
  }
}

int main(int argc, char** argv) {
  static const std::string canvas_name("my_canvas");
  ros::init(argc, argv, "trajectory_follower", ros::init_options::NoSigintHandler);
  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(argc, argv, &qapp);
  const ViewerCanvasPtr& canvas = viewer_core.getCanvas(canvas_name);
  srrg2_qgl_viewport::ViewerCoreSharedQGL::stop();

  std::thread node_t(node, canvas, argv);
  viewer_core.startViewerServer();
  node_t.join();
  return 0;
}
