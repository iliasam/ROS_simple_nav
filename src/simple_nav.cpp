//Taken from https://gist.github.com/TimSC/
//local target point - LTP, point of the path where robot is going

#include <ros/ros.h>
#include "simple_nav.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include <global_planner/planner_core.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <stdio.h>
#include "PathFilter.h"
#include "PathProcessing.h"
#include "MoveProcessing.h"

//Timer's frequencies
double move_update_freq;//Movement direction update frequency, Hz
double g_path_update_freq;//global path update frequency, Hz

tf::TransformListener* tf_tree_p;
costmap_2d::Costmap2DROS* my_costmap_p;
global_planner::GlobalPlanner my_global_planner;
bool verbose;

bool active_goal_present = false; //robot have goal to move


std::vector <geometry_msgs::PoseStamped> global_plan;
geometry_msgs::PoseStamped cur_robot_pos; //Current robot position
geometry_msgs::PoseStamped goal;  //Destination point for the global plan


//Working with created plan
PathFilter my_path_filter; //Path filtering object
PathProcessing my_path_processing;

ros::Publisher path_publisher;//Simplified path publisher
nav_msgs::Path simple_path;
ros::Publisher marker_publisher;

//Moving
MoveProcessing my_move_processing;

ros::Time move_start_time;


//Functions prototypes

void movement_timer_callback(const ros::TimerEvent& event);
void global_path_update_timer_callback(const ros::TimerEvent& event);
void goal_received_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void publish_LTP_marker(bool path_ok);
void update_cur_robot_position(void);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_nav_node");
    ros::NodeHandle priv_nh("~");


    priv_nh.param<double>("move_update_freq", move_update_freq, 5.0);
    priv_nh.param<double>("global_path_update_freq", g_path_update_freq, 1.0);

    priv_nh.setParam("my_path_planner/use_dijkstra", true);

    priv_nh.param<double>("filter_epsilon", my_path_filter.filter_epsilon, 0.05);//level of global path filtering (bigger value - stronger filtering)

    priv_nh.param<double>("min_angular_speed", my_move_processing.min_anglar_speed, 0.15);
    priv_nh.param<double>("max_angular_speed", my_move_processing.max_anglar_speed, 0.45);
    priv_nh.param<double>("angular_speed_p_coef", my_move_processing.angular_speed_p_coef, 0.015);//coefficient of increasing rotation speed
    priv_nh.param<double>("min_yaw_err_threshold", my_move_processing.min_yaw_err_threshold, 7.0);
    priv_nh.param<double>("min_linear_speed", my_move_processing.min_linear_speed, 0.2);

    priv_nh.param<double>("goal_dist_accur", my_path_processing.goal_dist_accur, 0.1);
    priv_nh.param<bool>("verbose", verbose, true);

    my_move_processing.MoveProcessingInit(&priv_nh);



    path_publisher    = priv_nh.advertise<nav_msgs::Path>("simple_path", 1);
    marker_publisher  = priv_nh.advertise<visualization_msgs::Marker>("vis_marker", 10);
    ros::Subscriber goal_sub = priv_nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goal_received_callback);

    ros::Timer movement_timer       = priv_nh.createTimer(ros::Duration(1.0 / move_update_freq), movement_timer_callback, false, false);//timer wouldn't work without running ros time!
    ros::Timer g_path_update_timer  = priv_nh.createTimer(ros::Duration(1.0 / g_path_update_freq), global_path_update_timer_callback, false, false);

    try
    {
        ros::Rate loop_rate(10);
        ROS_INFO("Simple Navigation started");

        tf_tree_p = new tf::TransformListener(ros::Duration(10));
        my_costmap_p = new costmap_2d::Costmap2DROS("global_costmap", *tf_tree_p);
        my_global_planner.initialize("my_path_planner", my_costmap_p);

        //default goal
        goal.header.frame_id = my_costmap_p->getGlobalFrameID();
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = -3.0;
        goal.pose.position.y = -3.0;
        goal.pose.position.z = 0;
        goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        simple_path.header.frame_id = my_costmap_p->getGlobalFrameID();

        movement_timer.start();
        g_path_update_timer.start();

        //active_goal_present = true; //debug
        move_start_time = ros::Time::now();


        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
    }
    catch (int)
    {
        ROS_ERROR("Error\n");
        return -1;
    }
}

void movement_timer_callback(const ros::TimerEvent& event)
{
    if (active_goal_present)
    {
        update_cur_robot_position();
        my_path_processing.UpdateLTP();
        uint8_t LTV_state = my_path_processing.CheckLTVState(cur_robot_pos);//needed YAW/DIST changed
        int16_t yaw_err = my_path_processing.CheckRobotYaw(cur_robot_pos);

        if (my_path_processing.path_is_actual)
        {
            int goal_result = my_path_processing.CheckArrivalStatus(cur_robot_pos);
            if (goal_result == 1)
            {
                //robot has arrived to goal pose
                active_goal_present = false;
                my_move_processing.movement_enabled = false;
                my_path_processing.path_is_actual = false;
                ROS_INFO("Destination reached!");

                ros::Duration duration = ros::Time::now() - move_start_time;
                double dur_sec = duration.toSec();
                ROS_INFO("Movement time: %d sec", (int)dur_sec);
            }
        }

        my_move_processing.CalculateRobotSpeed(yaw_err);
        if (verbose)
            ROS_INFO("YAW error: %d", yaw_err);
    }
}

//Create new global path
//Filters it and publish the result
void global_path_update_timer_callback(const ros::TimerEvent& event)
{
    if (active_goal_present == true)
    {
        update_cur_robot_position();

        bool res = my_global_planner.makePlan(cur_robot_pos, goal, global_plan);
        if (res)
        {
            int psize = (int)global_plan.size();
            if (verbose)
            {
                ROS_INFO("Global path found.");
                ROS_INFO("Raw path size: %d", psize);
            }

            my_path_filter.FilterRosPath(global_plan, &simple_path);
            my_path_processing.UpdateCurrentPath(&simple_path);

            path_publisher.publish(simple_path);
            my_move_processing.movement_enabled = true;
        }
        else
        {
            my_move_processing.movement_enabled = false;
            my_path_processing.path_is_actual = false;
			ROS_INFO("Global path not found");
        }
        my_path_processing.UpdateLTP();
        publish_LTP_marker(my_path_processing.path_is_actual);
    }
    else
    {
        publish_LTP_marker(false);
    }
}

//Callback called when goal position is received by this node
void goal_received_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    active_goal_present = true;
    move_start_time = ros::Time::now();

    ROS_INFO("Goal received");
}

//Update current robot position
void update_cur_robot_position(void)
{
    tf::StampedTransform cur_transform;
    try
    {
        tf_tree_p->lookupTransform("/map", "/base_link", ros::Time(0), cur_transform);

        cur_robot_pos.header.frame_id = my_costmap_p->getGlobalFrameID();
        cur_robot_pos.header.stamp = ros::Time::now() ;
        cur_robot_pos.pose.position.x = cur_transform.getOrigin().x();
        cur_robot_pos.pose.position.y = cur_transform.getOrigin().y();
        cur_robot_pos.pose.position.z = 0;

        geometry_msgs::Quaternion quat;  //geometry_msgs object for quaternion
        tf::Quaternion tfQuat = cur_transform.getRotation();
        quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
        quat.y = tfQuat.y();
        quat.z = tfQuat.z();
        quat.w = tfQuat.w();

        cur_robot_pos.pose.orientation = quat;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
}

//Publish arrow pointing to LTP
void publish_LTP_marker(bool path_ok)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = my_costmap_p->getGlobalFrameID();
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "LTP_line";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::ARROW;
    line_list.scale.x = 0.02;//shaft diameter
    line_list.scale.y = 0.05;//head diameter
    line_list.scale.z = 0.15;//head length

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;//alpha

    if (path_ok)
    {
        line_list.points.push_back(cur_robot_pos.pose.position);
        line_list.points.push_back(my_path_processing.LTP_position);
        marker_publisher.publish(line_list);
    }
}
