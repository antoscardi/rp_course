Header header

#command input to the collision avoider
geometry_msgs/Twist cmd_vel_input

#command output by the collision avoider
geometry_msgs/Twist cmd_vel_output


# status can be either
# "clear":    when the obstacles are fare enough and the avoider does not kick in
# "adjusting": when the obstacles affect the control, but don't block the robot
# "blocked":   when the robot stopped due to a dynamic obstacle

string status