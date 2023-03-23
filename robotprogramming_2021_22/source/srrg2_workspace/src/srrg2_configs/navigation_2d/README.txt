2D navigation stack

Map_server
$> rosrun srrg2_map_server map_server <yaml map file>

Localizer:
$> ./localizer_2d.srrg

Planner:
$> ./planner_2d.srrg

Path follower (static)
rosrun srrg2_navigation_2d_ros path_follower_app _path_topic:=/path

Path follower (dynamic)
srrg2_navigation_2d_ros/path_follower_app _path_topic:=/local_path


Alternatively
1. install srrg2_webctl (if u use 20.04, libwebsocket has changed and you need to set the right branch)

2. after setting the environment, run
$> proc_webctl run_navigation.srrg

3. start a browser on localhost:9001, and start the programs
