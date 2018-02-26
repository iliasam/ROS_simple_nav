# simple_nav
This is a simple navigation node for the ROS.  
There is no local planner been used, so that can cause robot bump to some obstacles, which does not present at the map.
This node periodically create global plan to the goal and filter it.  
Robot simply goes by the points of this plan.  

This node was tested at Orange Pi PC with hector_slam running. The performance of this device was enough to run all navigation.  
Example of working: https://www.youtube.com/watch?v=W3qkPr2p2U0 and another one: https://www.youtube.com/watch?v=jN23EdEAeXU

Это простой узел навигации для ROS.  
Здесь не использутся локальный планировщик пути, поэтому робот может сталкиваться с объектами, которых нет на карте.  
Этот узел периодически прокладывает глобальный путь от робота до целевой точки и производит его фильтрацию, так что число точек в пути значительно сокращается.  
Затем робот просто следует ко второй точке в этом пути (первой точкой являются стартовые координаты робота).  

Этот узел был проверен на Orange Pi PC совместно с hector_slam. Производительности этого компьютера достаточно, чтобы поддерживать работу вей навигации.

### Subscribed Topics
* "/move_base_simple/goal" <geometry_msgs/PoseStamped> - goal point.
* "/map" <nav_msgs/OccupancyGrid>
* "/tf" <tf/tfMessage>  
/map → /base_link (Usually provided by a node responsible for odometry or localization).

### Published Topics
* <nav_msgs/OccupancyGrid>
* "/simple_path" <nav_msgs/Path> - global path for the robot.
* "/vis_marker" <visualization_msgs/Marker> - marker (arrow) to the local target point of the robot.
* "/cmd_vel" <geometry_msgs/Twist> - speed commands for the robot.

### Parameters
* ~filter_epsilon (double, default: 0.05)   
Level of global path filtering (bigger value - stronger filtering)
* ~min_angular_speed (double, default: 0.15)  
Minimum angular speed of the robot.
* ~max_angular_speed (double, default: 0.45)  
Maximum angular speed of the robot.
* ~angular_speed_p_coef (double, default: 0.015)  
Coefficient of increasing rotation speed - bigger yaw error → bigger angular speed of the robot.
* ~min_yaw_err_threshold (double, default: 7.0)  
Angle in deg. If the yaw error of the robot is less than this value, robot begin linear movement.
* ~min_linear_speed (double, default: 0.2)  
Now this is single linear speed for the robot.
* ~goal_dist_accur (double, default: 0.1)  
If the distance between the current robot pose and the goal is less than this value than robot stops.

Some parameters of "costmap_2d" are stored in "/bin/costmap_params.yaml" file.


