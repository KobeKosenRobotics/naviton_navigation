#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <nav_msgs/Path.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "waypoint_manager/waypoint_recorder_start.h"
#include "waypoint_manager/waypoint_recorder_end.h"

class WaypointRecorder
{
    public:
        WaypointRecorder(ros::NodeHandle &nh, ros::NodeHandle &pn);
    private:
        bool recorder_start_cb(waypoint_manager::waypoint_recorder_start::Request& req, waypoint_manager::waypoint_recorder_start::Response& res);
        bool recorder_end_cb(waypoint_manager::waypoint_recorder_end::Request& req, waypoint_manager::waypoint_recorder_end::Response& res);
        void record();

        ros::ServiceServer _recorder_start_server;
        ros::ServiceServer _recorder_end_server;

        ros::Timer _timer;

        nav_msgs::Path _path;
        
        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        geometry_msgs::TransformStamped _tf_stamped;
        geometry_msgs::PoseStamped _pose_stamped;

        std::string _frame_id_map;
        std::string _frame_id_robot;

        bool _recording = false;
        geometry_msgs::Point _point_old;

        std::string _file_dir;
        std::string _file_name;
        double _pitch2;
};

#endif