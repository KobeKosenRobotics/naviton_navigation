#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <waypoint_manager_msgs/waypoint_recorder_start.h>
#include <waypoint_manager_msgs/waypoint_recorder_end.h>
#include <waypoint_manager_msgs/waypoint_recorder_record.h>
#include <waypoint_manager_msgs/waypoint_recorder_attributes.h>

class WaypointRecorder
{
    struct Attribute
    {
        std::string attribute;
        double value;
    };

    public:
        WaypointRecorder(ros::NodeHandle &nh, ros::NodeHandle &pn);
    private:
        bool recorder_start_cb(waypoint_manager_msgs::waypoint_recorder_start::Request& req, waypoint_manager_msgs::waypoint_recorder_start::Response& res);
        bool recorder_end_cb(waypoint_manager_msgs::waypoint_recorder_end::Request& req, waypoint_manager_msgs::waypoint_recorder_end::Response& res);
        bool recorder_record_cb(waypoint_manager_msgs::waypoint_recorder_record::Request& req, waypoint_manager_msgs::waypoint_recorder_record::Response& res);
        bool recorder_attributes_cb(waypoint_manager_msgs::waypoint_recorder_attributes::Request& req, waypoint_manager_msgs::waypoint_recorder_attributes::Response& res);
        void record();
        void record(std::vector<Attribute> attributes);

        ros::ServiceServer _recorder_start_server;
        ros::ServiceServer _recorder_end_server;
        ros::ServiceServer _recorder_record_server;
        ros::ServiceServer _recorder_attributes_server;

        ros::Timer _timer;

        std::vector<geometry_msgs::PoseStamped> _poses;
        std::vector<std::vector<Attribute>> _attributes;
        
        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        geometry_msgs::TransformStamped _tf_stamped;
        geometry_msgs::PoseStamped _pose_stamped;

        double _lookupTimeout;
    
        std::string _frame_id_map;
        std::string _frame_id_robot;

        std::vector<Attribute> _attirbutes_permanent;

        bool _recording = false;
        geometry_msgs::Point _point_old;

        std::string _file_dir;
        double _pitch2;
};

#endif