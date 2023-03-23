Header header

#how many steps we need to reach the goal
int64 num_steps_to_goal 

#2D robot pose (x,y, theta)
float32[3] robot_pose_2d

#2D target of the robot in the local frame
float32[3] local_target_2d

#command output by the follower
geometry_msgs/Twist cmd_vel

# status can be either
# "goal_reached":    when the path is empty
# "initial_turning": when the robot initially rotates to align with the path
# "cruising":        when the robot is tracking the path with the regular controller
# "finalizing":      when the robot reached the final position, and adjusts the orientation
# "error":           when the robot cannot reach the goal because no local target could be computed 
string status