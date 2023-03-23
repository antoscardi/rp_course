# 2D really simple navigation package that works decently

## How to run with the provided shit

### ======================= INSTALLING ============================
Compile srrg shit then run

`sudo apt remove ros-melodic-navigation`


### ======================= RUNNING ============================

after, enter the config folder, edit the `dl.conf` and open a bunch of terminals in this folders


#### TX: stands for terminal x

T1 
```roscore```

T2
```rosrun stage_ros stageros cappero_laser_odom_diag_2020-05-06-16-26-03.world```

T3 source the srrg workspace (our version of the map_server is not the same as the ROS one) then run
```rosrun srrg2_map_server map_server cappero_laser_odom_diag_2020-05-06-16-26-03.yaml```

T4 `rviz`

T5 source the srrg workspace first, then run
```rosrun srrg2_executor srrg2_shell run_localizer_live_ms.srrg```

T6 source the srrg workspace first, then run
```rosrun srrg2_executor srrg2_shell  run_planner_live_ms.srrg```

#### -------------------- path following with no obstacle avoidance -----------------------

T7 source the srrg workspace first, then run
```rosrun srrg2_navigation_2d_ros path_follower```

#### -------------------- path following with obstacle avoidance --------------------------

T7 source the srrg workspace first, then run
```rosrun srrg2_navigation_2d_ros path_follower _cmd_vel_topic:=/cmd_vel_input```

T8 source the srrg workspace first, then run
```rosrun srrg2_navigation_2d_ros collision_avoider _cmd_vel_input_topic:=/cmd_vel_input```

#### -------------------- make some reproducible experiments ------------------------------
T9 source the srrg workspace first, then run
```rosrun srrg2_navigation_2d_ros path_provider _waypoint_filename:=<path/to/waypoints.txt> _path_iterations:=<number_of_iterations>```

### ============================ USING ==========================================

Localize manually the robot from rviz

         Global localization can be done in this map with
         - 2000/3000 particles
         - enabling the particle resetting parameter in the localizer (use the shell)
         Once localized disable particle resetting;
         We don't need global localization now so don't waste time
         
Set a goal from rviz

watch

To stop the robot set a goal in the unknown space


  


