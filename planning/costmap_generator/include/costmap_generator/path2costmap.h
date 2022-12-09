#ifndef PATH2COSTMAP_H
#define PATH2COSTMAP_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

class Path2Costmap
{
    public:
        Path2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        void path_cb(const nav_msgs::Path::ConstPtr& path_msg);

        ros::Subscriber _path_sub;
        ros::Publisher _costmap_pub;

        nav_msgs::OccupancyGrid _costmap;
};

#endif