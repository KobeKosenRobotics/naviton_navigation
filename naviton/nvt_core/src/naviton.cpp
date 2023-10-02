#include "nvt_core/naviton.h"

Naviton::Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string service_wpManager_set, topic_nowWp_local;
    
    pn.param<std::string>("service_wpManager_set", service_wpManager_set, "/naviton/waypoint/wpManager/set");
    pn.param<std::string>("topic_nowWp_local", topic_nowWp_local, "/naviton//waypoint/wpManager/nowWp_local");

    _wpManager_set_client = nh.serviceClient<waypoint_manager_msgs::waypoint_manager_set>(service_wpManager_set);
    _nowWp_local_subscriber = nh.subscribe(topic_nowWp_local, 10, &Naviton::nowWp_local_cb, this);
}

void Naviton::init()
{
    _innerIndex = 0;
}

void Naviton::update()
{
    std::cout << _innerIndex << std::endl;
    if(_innerIndex == -1) return;

    if(_innerIndex != _nowWp_local.index)
    {
        waypoint_manager_msgs::waypoint_manager_set srv;
        srv.request.index = _innerIndex;
        _wpManager_set_client.call(srv);
        return;
    }

    geometry_msgs::Point point = _nowWp_local.pose.pose.position;
    double distance_sqr = point.x*point.x+point.y*point.y+point.z*point.z;
    if(distance_sqr < 4.0)
    {
        _innerIndex++;
    }
}

void Naviton::nowWp_local_cb(waypoint_msgs::waypointConstPtr msg)
{
    _nowWp_local = *msg;
}