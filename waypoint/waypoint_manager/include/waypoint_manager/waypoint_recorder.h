#ifndef WAYPOINT_RECORDER_H
#define WAYPOINT_RECORDER_H

#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <waypoint_msgs/waypoints.h>
#include <waypoint_manager_msgs/waypoint_recorder_start.h>
#include <waypoint_manager_msgs/waypoint_recorder_end.h>
#include <waypoint_manager_msgs/waypoint_recorder_record.h>
#include <waypoint_manager_msgs/waypoint_recorder_attributes.h>

class WaypointRecorder
{
    public:
        WaypointRecorder(ros::NodeHandle &nh, ros::NodeHandle &pn);
    private:
        bool recorder_start_cb(waypoint_manager_msgs::waypoint_recorder_start::Request& req, waypoint_manager_msgs::waypoint_recorder_start::Response& res);
        bool recorder_end_cb(waypoint_manager_msgs::waypoint_recorder_end::Request& req, waypoint_manager_msgs::waypoint_recorder_end::Response& res);
        void save(std::string file_name);
        bool recorder_record_cb(waypoint_manager_msgs::waypoint_recorder_record::Request& req, waypoint_manager_msgs::waypoint_recorder_record::Response& res);
        bool recorder_attributes_cb(waypoint_manager_msgs::waypoint_recorder_attributes::Request& req, waypoint_manager_msgs::waypoint_recorder_attributes::Response& res);
        void record();
        void record(std::vector<waypoint_msgs::waypoint_attribute> attributes);
        void publishPath();

        ros::Publisher _wps_publisher;
        ros::ServiceServer _recorder_start_server;
        ros::ServiceServer _recorder_end_server;
        ros::ServiceServer _recorder_record_server;
        ros::ServiceServer _recorder_attributes_server;

        ros::Timer _timer;

        waypoint_msgs::waypoints _wps;
        int _wpIndex;
        
        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        geometry_msgs::TransformStamped _tf_stamped;
        geometry_msgs::PoseStamped _pose_stamped;

        double _lookupTimeout;
    
        std::string _frame_id_map;
        std::string _frame_id_robot;

        bool _recording = false;
        geometry_msgs::Point _point_old;

        std::string _file_dir;
        double _pitch2;

        std::vector<waypoint_msgs::waypoint_attribute> _attributes_permanent;
};

#endif