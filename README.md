#simple_nav
This is a simple navigation node for the ROS.
There is no local planner been used, so that can cause robot bump to some obstacles, which does not present at the map.

Subscribed Topics
* "/move_base_simple/goal" <geometry_msgs::PoseStamped> - goal point.
* "/map"

Published Topics
* <nav_msgs/OccupancyGrid>
* "/simple_path" <nav_msgs::Path> - global path for the robot.
* "/vis_marker" <visualization_msgs::Marker> - marker (arrow) to the local target point of the robot.
* "/cmd_vel" <geometry_msgs::Twist> - speed commands for the robot.

Required tf Transforms
/map → /base_link
Usually provided by a node responsible for odometry or localization