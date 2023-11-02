#ifndef NAVITON_H
#define NAVITON_H

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <waypoint_msgs/waypoint.h>
#include <waypoint_manager_msgs/waypoint_manager_set.h>

class Naviton
{
    public:
        Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void init();
        void update();
        void nowWp_local_cb(waypoint_msgs::waypointConstPtr msg);

        bool start_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        bool pause_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    private:
        ros::ServiceServer _nvt_start_server;
        ros::ServiceServer _nvt_pause_server;
        ros::ServiceClient _wpManager_set_client;
        ros::Subscriber _nowWp_local_subscriber;
        waypoint_msgs::waypoint _nowWp_local;

        bool _paused;
};

#endif