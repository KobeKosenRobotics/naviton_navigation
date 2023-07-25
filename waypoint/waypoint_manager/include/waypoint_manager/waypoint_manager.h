#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/exceptions.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <waypoint_manager_msgs/waypoint_manager_set.h>
#include <waypoint_msgs/waypoints.h>

class WaypointManager
{
    public:
        WaypointManager(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void publish();
    private:
        bool manager_set_cb(waypoint_manager_msgs::waypoint_manager_set::Request& req, waypoint_manager_msgs::waypoint_manager_set::Response& res);
        void waypoints_cb(waypoint_msgs::waypointsConstPtr msg);

        int64_t _index_now = -1;
        waypoint_msgs::waypoints _wps;

        ros::Publisher _nowWp_publisher;
        ros::Publisher _nowWpPose_publisher;
        ros::Subscriber _wps_subscriber;
        ros::ServiceServer _manager_set_server;

        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        double _lookupTimeout;
        std::string _frame_id_robot;
        std::string _frame_id_map;
};

#endif