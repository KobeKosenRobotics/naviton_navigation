#ifndef WAYPOINT_VISUALIZER_H
#define WAYPOINT_VISUALIZER_H

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <waypoint_msgs/waypoint.h>
#include <waypoint_msgs/waypoints.h>

class WaypointVisualizer
{
    public:
        WaypointVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void publish();
    private:
        visualization_msgs::MarkerArray generateMarkerArray();
        void waypoints_cb(waypoint_msgs::waypointsConstPtr msg);

        ros::Publisher _markerArray_publisher;
        ros::Subscriber _wps_subscriber;

        waypoint_msgs::waypoints _wps;
};

#endif