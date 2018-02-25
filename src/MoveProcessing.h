#ifndef MOVEPROCESSING_H
#define MOVEPROCESSING_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class MoveProcessing
{
    public:
        bool movement_enabled;
        double min_anglar_speed;
        double max_anglar_speed;
        double angular_speed_p_coef;
        double min_yaw_err_threshold;//deg
        double min_linear_speed;

        MoveProcessing();
        virtual ~MoveProcessing();
        void MoveProcessingInit(ros::NodeHandle* node_handle_p);
        void CalculateRobotSpeed(int16_t cur_yaw_error);
    protected:
    private:
        void speed_update_timer_callback(const ros::TimerEvent& event);
        double calculate_angular_speed(int16_t yaw_error);
};

#endif // MOVEPROCESSING_H
