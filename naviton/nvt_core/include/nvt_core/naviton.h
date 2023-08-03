#ifndef NAVITON_H
#define NAVITON_H

#include <ros/ros.h>

#include <waypoint_msgs/waypoint.h>
#include <waypoint_manager_msgs/waypoint_manager_set.h>

class Naviton
{
    public:
        Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void init();
        void update();
        void nowWp_local_cb(waypoint_msgs::waypointConstPtr msg);
    private:
        ros::ServiceClient _wpManager_set_client;
        ros::Subscriber _nowWp_local_subscriber;
        waypoint_msgs::waypoint _nowWp_local;

        int _innerIndex;
};

#endif