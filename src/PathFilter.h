#ifndef PATHFILTER_H
#define PATHFILTER_H

#include <utility> //for std::pair
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

typedef std::pair<double, double> Point;

class PathFilter
{
    public:
        double filter_epsilon;

        PathFilter(void);
        virtual ~PathFilter(void);
        void FilterRosPath(std::vector<geometry_msgs::PoseStamped>& in_path, nav_msgs::Path* out_path);
    protected:
    private:
        double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd);
        void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);
};

#endif // PATHFILTER_H
