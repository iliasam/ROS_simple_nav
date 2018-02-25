#include "MoveProcessing.h"
#include <geometry_msgs/Twist.h>

double speed_update_freq = 10.0;//Robot speed update frequency, Hz

ros::Publisher cmd_vel_publisher;
ros::Timer speed_update_timer;


double cur_anglar_speed = 0.0;
double cur_linear_speed = 0.0;//m/sec

//Error in degrees
#define MIN_ANGULAR_ROTATION_ERROR      2

//rotation during linear movement - rad/sec
#define MICRO_ANGULAR_ROTATION_SPEED    0.05


MoveProcessing::MoveProcessing()
{
    movement_enabled = false;
    //ctor
}

MoveProcessing::~MoveProcessing()
{
    //dtor
}


void MoveProcessing::MoveProcessingInit(ros::NodeHandle* node_handle_p)
{
    cmd_vel_publisher = node_handle_p->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    speed_update_timer = node_handle_p->createTimer(ros::Duration(1 / speed_update_freq), &MoveProcessing::speed_update_timer_callback, this);
}

//External software send difference from needed yaw in deg
//This funstion set "cur_anglar_speed" and "cur_linear_speed"
void MoveProcessing::CalculateRobotSpeed(int16_t cur_yaw_error)
{
    bool yaw_is_good = true;//true if the robot is oriented for linear movement

    if (cur_yaw_error > min_yaw_err_threshold)
    {
        cur_anglar_speed = -MoveProcessing::calculate_angular_speed(cur_yaw_error);
        yaw_is_good = false;
    }
    else if (cur_yaw_error < -min_yaw_err_threshold)
    {
        cur_anglar_speed = -MoveProcessing::calculate_angular_speed(cur_yaw_error);
        yaw_is_good = false;
    }
    else if (cur_yaw_error > MIN_ANGULAR_ROTATION_ERROR)
        cur_anglar_speed = -MICRO_ANGULAR_ROTATION_SPEED;//rotation during linear movement
    else if (cur_yaw_error < -MIN_ANGULAR_ROTATION_ERROR)
        cur_anglar_speed = MICRO_ANGULAR_ROTATION_SPEED;//rotation during linear movement
    else
        cur_anglar_speed = 0.0;


    if (yaw_is_good == true)
    {
        cur_linear_speed = min_linear_speed;
    }
    else
    {
        cur_linear_speed = 0.0;//robot must stop linear movement while yaw correction
        ROS_INFO("ROTATION");
    }

}

double  MoveProcessing::calculate_angular_speed(int16_t yaw_error)
{
    double ang_speed = yaw_error * angular_speed_p_coef;

    if ((ang_speed < min_anglar_speed) && (ang_speed > 0.0))
        return min_anglar_speed;

    if ((ang_speed < 0.0) && (ang_speed > -min_anglar_speed))
        return -min_anglar_speed;

    return ang_speed;
}

//Publish movement commands for the robot
void MoveProcessing::speed_update_timer_callback(const ros::TimerEvent& event)
{
    geometry_msgs::Twist cmd_vel;

    if (movement_enabled == true)
    {
        cmd_vel.linear.x = cur_linear_speed;
        cmd_vel.angular.z = cur_anglar_speed;
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    if (cmd_vel.angular.z > max_anglar_speed)
        cmd_vel.angular.z = max_anglar_speed;
    else if (cmd_vel.angular.z < -max_anglar_speed)
        cmd_vel.angular.z = -max_anglar_speed;

    cmd_vel_publisher.publish(cmd_vel);
}

