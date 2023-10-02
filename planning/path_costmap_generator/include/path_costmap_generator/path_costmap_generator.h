#ifndef PATH_COSTMAP_GENERATOR_H
#define PATH_COSTMAP_GENERATOR_H

#include <omp.h>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <waypoint_msgs/waypoint.h>
#include <waypoint_msgs/waypoints.h>

class PathCostmapGenerator
{
    public:
        PathCostmapGenerator(ros::NodeHandle &nh, ros::NodeHandle &pn);
        
        void update();
    private:
        void waypoint_cb(waypoint_msgs::waypointConstPtr msg);
        void waypoints_cb(waypoint_msgs::waypointsConstPtr msg);

        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        nav_msgs::OccupancyGrid _costmap;

        ros::Subscriber _wp_sub;
        ros::Subscriber _wps_sub;

        int _wpIndex_now;
        waypoint_msgs::waypoints _wps;

        float _width;
        float _resolution;
        int _grid_width;
        int _grid_num;
        
        float _width_2;
        int _grid_width_2;

        bool _wp_subscribed;
        bool _wps_subscribed;
};

#endif