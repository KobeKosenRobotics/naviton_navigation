#ifndef WAYPOINT_LOADER_H
#define WAYPOINT_LOADER_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <waypoint_msgs/waypoints.h>

class WaypointLoader
{
    public:
        WaypointLoader(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void load(std::string file_path);
        void publish();
    private:
        ros::Publisher _wps_publisher;

        waypoint_msgs::waypoints _wps;
        std::string _frame_id_map;
};

#endif