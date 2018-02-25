#include "PathProcessing.h"
#include <angles/angles.h>
#include <tf/transform_listener.h>

//local target point - LTP, point of the path were robot is going
//local target vector - LTV, vector of the robot movement

double cur_LTV_yaw;//Nedded yaw of the robot - in radians
double prev_LTV_yaw;//Previous "LTV_yaw"
geometry_msgs::Point prev_LTP_position;

//Points in the path goes from the robot to the goal
nav_msgs::Path internal_path;



PathProcessing::PathProcessing()
{
    path_is_actual = false;
    goal_dist_accur = 0.1;
}

PathProcessing::~PathProcessing()
{
    //dtor
}

//Calculate new position of LTP, depending on current position of the robot
void PathProcessing::UpdateLTP(void)
{
    int LTP_number = 0;//number of point in the "internal_path" from the end
    if (path_is_actual == false)
        return;

    int path_size = internal_path.poses.size();

    if (path_size == 0)
    {
        path_is_actual = false;
        return; //error
    }
    else if (path_size == 1) //There is only one point in path
        LTP_number = 0;
    else if (path_size == 2) //Two points in the path, second is the needed
        LTP_number = 1;
    else
    {
        LTP_number = 1;//Second point from the path start (it's not a good method)
    }

    LTP_position = internal_path.poses[LTP_number].pose.position;

    //ROS_INFO("size: %d", path_size);
}

//Detect if needed local target vector YAW/DIST changed
//robot_pos - current robot position
uint8_t PathProcessing::CheckLTVState(geometry_msgs::PoseStamped robot_pos)
{
    bool LTV_yaw_changed = false;
    bool LTP_pos_changed = false;

    //calculate vector angle
    double LTV_dx = (LTP_position.x - robot_pos.pose.position.x);
    double LTV_dy = (LTP_position.y - robot_pos.pose.position.y);

    cur_LTV_yaw = atan2(LTV_dy, LTV_dx);//Nedded yaw of the robot
    //double LTV_yaw_deg = angles::to_degrees(cur_LTV_yaw);
    //ROS_INFO("angle: %f", LTV_yaw_deg);

    double tmp_LTV_diff = angles::shortest_angular_distance(cur_LTV_yaw, prev_LTV_yaw);
    double tmp_TTV_diff_deg = angles::to_degrees(tmp_LTV_diff);

    if (abs(tmp_TTV_diff_deg) > 6.0)
        LTV_yaw_changed = true;

    double LTP_diff = PointsDistance(LTP_position.x, LTP_position.y, prev_LTP_position.x, prev_LTP_position.y);
    if (LTP_diff > 0.1)
        LTP_pos_changed = true;

    prev_LTV_yaw = cur_LTV_yaw;
    prev_LTP_position = LTP_position;

    if (LTV_yaw_changed)
    {
        //ROS_INFO("YAW changed\n");
        return 1;
    }


    if (LTP_pos_changed)
    {
        //ROS_INFO("DIST changed\n");
        return 2;
    }

    return 0;
}

//Check if the robot Yaw is good
//robot_pos - current robot position
//Return the difference on robot current yaw and needed yaw in degrees
int16_t PathProcessing::CheckRobotYaw(geometry_msgs::PoseStamped robot_pos)
{
    double cur_robot_yaw = tf::getYaw(robot_pos.pose.orientation);
    //double cur_robot_yaw_deg = angles::to_degrees(cur_robot_yaw);

    double tmp_LTV_diff = angles::shortest_angular_distance(cur_LTV_yaw, cur_robot_yaw);
    double tmp_TTV_diff_deg = angles::to_degrees(tmp_LTV_diff);

    return (int16_t)tmp_TTV_diff_deg;
}

//new_path_p - pointer to filtered global path
void PathProcessing::UpdateCurrentPath(nav_msgs::Path* new_path_p)
{
    int new_path_size = new_path_p->poses.size(); //number of points
    memcpy(&internal_path, new_path_p, sizeof(internal_path));//copy to internal buffer

    if (new_path_size == 0)
        path_is_actual = false;

    path_is_actual = true;
}

//Check if robot already arrived to destination
//robot_pos - current robot position
//return 1 if the goal is reached
int PathProcessing::CheckArrivalStatus(geometry_msgs::PoseStamped robot_pos)
{
    int path_size = internal_path.poses.size();
    if (path_size < 1)
        return -2;

    if (path_size > 3)
        return -1;//Goal is too far to check

    geometry_msgs::Point goal_point = internal_path.poses[path_size-1].pose.position;//last point of the path

    double length_to_goal = PointsDistance(robot_pos.pose.position.x, robot_pos.pose.position.y, goal_point.x, goal_point.y);

    if (length_to_goal < goal_dist_accur)
        return 1;//robot reached goal

    //ROS_INFO("dist to goal: %f", length_to_goal);
    return 0;
}

double PathProcessing::PointsDistance(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx*dx + dy*dy);
}
