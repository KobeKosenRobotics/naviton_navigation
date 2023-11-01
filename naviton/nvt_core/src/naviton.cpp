#include "nvt_core/naviton.h"

Naviton::Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string service_wpManager_set, topic_nowWp_local;
    
    pn.param<std::string>("service_wpManager_set", service_wpManager_set, "/naviton/waypoint/wpManager/set");
    pn.param<std::string>("topic_nowWp_local", topic_nowWp_local, "/naviton/waypoint/wpManager/nowWp_local");

    _wpManager_set_client = nh.serviceClient<waypoint_manager_msgs::waypoint_manager_set>(service_wpManager_set);
    _nowWp_local_subscriber = nh.subscribe(topic_nowWp_local, 10, &Naviton::nowWp_local_cb, this);
}

void Naviton::init()
{
}

void Naviton::update()
{
    geometry_msgs::Point point = _nowWp_local.pose.pose.position;
    double distance_sqr = point.x*point.x+point.y*point.y+point.z*point.z;
    if(distance_sqr < 4.0)
    {
        waypoint_manager_msgs::waypoint_manager_set srv;

        auto next_waypoint_attribute
            = std::find_if(_nowWp_local.attributes.begin(), _nowWp_local.attributes.end(),
            [](waypoint_msgs::waypoint_attribute &attribute)
            {
                return(attribute.type == attribute.TYPE_NEXT_WAYPOINT);
            } );

        if(next_waypoint_attribute != _nowWp_local.attributes.end())
        {
            srv.request.index = std::round(next_waypoint_attribute->value);
        }
        else
        {
            srv.request.index = _nowWp_local.index + 1;
        }
        _wpManager_set_client.call(srv);
    }
}

void Naviton::nowWp_local_cb(waypoint_msgs::waypointConstPtr msg)
{
    _nowWp_local = *msg;
}