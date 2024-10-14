#rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[4.9,0,0]' '[0,0,0]'
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[4.9,0,0]' '[0,0,1.8]'
