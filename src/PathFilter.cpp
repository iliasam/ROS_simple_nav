#include "PathFilter.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

//using namespace std;


PathFilter::PathFilter(void)
{
    //ctor
    filter_epsilon = 0.05;
}

PathFilter::~PathFilter(void)
{
    //dtor
}


double PathFilter::PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
    double dx = lineEnd.first - lineStart.first;
    double dy = lineEnd.second - lineStart.second;

    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag;
        dy /= mag;
    }

    double pvx = pt.first - lineStart.first;
    double pvy = pt.second - lineStart.second;

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}


void PathFilter::RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out)
{
    if(pointList.size()<2)
    {
        ROS_ERROR("Not enough points to simplify path");
        return;
    }

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<Point> recResults1;
        vector<Point> recResults2;
        vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<Point> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker(firstLine, epsilon, recResults1);
        RamerDouglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
        {
            ROS_ERROR("Problem assembling output");
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}

//Simplifies given path - remove useless points
void PathFilter::FilterRosPath(std::vector<geometry_msgs::PoseStamped>& in_path, nav_msgs::Path* out_path)
{
    int i;
    int in_size = in_path.size();
    //geometry_msgs::PoseStamped poseStamped;
    Point tmp_point;

    vector<Point> pointList;
    vector<Point> pointListOut;

    for(i=0; i < in_size; i++)
    {
        tmp_point.first = in_path[i].pose.position.x;
        tmp_point.second = in_path[i].pose.position.y;
        pointList.push_back(tmp_point);
    }
    RamerDouglasPeucker(pointList, filter_epsilon, pointListOut);
    int out_size = pointListOut.size();
    ROS_INFO("new path size: %d", out_size);

    out_path->poses.resize(out_size);
    for(i=0; i < out_size; i++)
    {
        out_path->poses[i].pose.position.x = pointListOut[i].first;
        out_path->poses[i].pose.position.y = pointListOut[i].second;
    }
    out_path->header.stamp = ros::Time::now();

}

