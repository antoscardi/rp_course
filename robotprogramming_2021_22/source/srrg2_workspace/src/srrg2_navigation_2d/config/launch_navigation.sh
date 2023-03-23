#!/bin/bash
xterm -hold -e "roscore"&
sleep 3
CONFIG_DIR="/home/srrg/workspaces/srrg2/src/srrg2_navigation_2d/config"
WSSRRG2="source /home/srrg/workspaces/srrg2/devel/setup.bash"
xterm -hold -e "cd $CONFIG_DIR;rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world"&
sleep 3
xterm -hold -e "cd $CONFIG_DIR;$WSSRRG2;rosrun srrg2_map_server map_server cappero_laser_odom_diag_2020-05-06-16-26-03.yaml"&
sleep 3
xterm -hold -e "rviz -d $CONFIG_DIR/rviz_navigation2d.rviz"&
sleep 3
xterm -hold -e "cd $CONFIG_DIR;$WSSRRG2;rosrun srrg2_executor srrg2_shell -dlc dl.conf run_localizer_live_ms.srrg"&
