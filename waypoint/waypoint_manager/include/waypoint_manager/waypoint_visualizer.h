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
    private:
        void waypoints_cb(waypoint_msgs::waypointsConstPtr msg);

        ros::Subscriber _wps_subscriber_1;
        ros::Subscriber _wps_subscriber_2;
        ros::Publisher _markerArray_publisher;

        std::string _frame_id_map;
        double _text_size;
        double _z_offset;
};

#endif