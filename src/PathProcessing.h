#ifndef PATHPROCESSING_H
#define PATHPROCESSING_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class PathProcessing
{
    public:
        geometry_msgs::Point LTP_position;
        bool path_is_actual;
        double goal_dist_accur;//goal distance accuracy - if distance is smaller, than robot stop linear movement

        PathProcessing();
        virtual ~PathProcessing();
        void UpdateCurrentPath(nav_msgs::Path* new_path_p);
        void UpdateLTP(void);
        uint8_t CheckLTVState(geometry_msgs::PoseStamped robot_pos);
        double PointsDistance(double x1, double y1, double x2, double y2);
        int16_t CheckRobotYaw(geometry_msgs::PoseStamped robot_pos);
        int CheckArrivalStatus(geometry_msgs::PoseStamped robot_pos);
    protected:
    private:
};

#endif // PATHPROCESSING_H
