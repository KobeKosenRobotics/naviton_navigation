#include "waypoint_manager/waypoint_manager.h"

WaypointManager::WaypointManager(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_nowWp, topic_waypoints, service_manager_set;
    
    pn.param<std::string>("topic_nowWp", topic_nowWp, "wpManager/nowWp");
    pn.param<std::string>("topic_waypoints", topic_waypoints, "wpLoader/waypoints");
    pn.param<std::string>("service_manager_set", service_manager_set, "wpManager/set");

    _nowWp_publisher = nh.advertise<waypoint_msgs::waypoint>(topic_nowWp, 1);
    _wps_subscriber = nh.subscribe(topic_waypoints, 1, &WaypointManager::waypoints_cb, this);
    _manager_set_server = nh.advertiseService(service_manager_set, &WaypointManager::manager_set_cb, this);

    _index_now = -1;
}

void WaypointManager::publish()
{
    if(_index_now < 0 || _wps.waypoints.size() < _index_now) return;
    _nowWp_publisher.publish(_wps.waypoints[_index_now]);
}

bool WaypointManager::manager_set_cb(waypoint_manager_msgs::waypoint_manager_set::Request& req, waypoint_manager_msgs::waypoint_manager_set::Response& res)
{
    _index_now = req.index;
    return true;
}

void WaypointManager::waypoints_cb(waypoint_msgs::waypointsConstPtr msg)
{
    _wps = *msg;
}