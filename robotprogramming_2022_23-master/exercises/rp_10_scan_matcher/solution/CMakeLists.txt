cmake_minimum_required(VERSION 3.16)

project(scan_matcher)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Find Eigen3 Package (Required for ICP component)
find_package(Eigen3 REQUIRED)

# -------------------------------------
# ROS related configurations
# -------------------------------------

# Find catkin package with the associated required components:
# -roscpp :
# ROS headers for C/C++
# -sensor_msgs :
# Messages commonly used by sensors. We need sensor_msgs/LaserScan for 2D Lidar data
# -nav_msgs :
# Messages used by the ROS navigation stack. We need nav_msgs/Odometry for registering sensor's pose
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# Register the current package in the catkin environment.
catkin_package(
    INCLUDE_DIRS # Exported included paths for the package (in our case only the include folder)
    "include/"
    CATKIN_DEPENDS # Dependencies from other catkin packages
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# Create ICP library
add_library(icp SHARED src/icp/eigen_icp_2d.cpp)
target_include_directories(icp PUBLIC include/icp)
target_link_libraries(icp PUBLIC Eigen3::Eigen)

# Create scan_matcher node
add_executable(scan_matcher_node src/scan_matcher_node.cpp src/ros_bridge.cpp)
target_include_directories(scan_matcher_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(scan_matcher_node PRIVATE ${catkin_LIBRARIES} icp)
