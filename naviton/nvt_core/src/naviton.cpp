#include "nvt_core/naviton.h"

Naviton::Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string service_wpManager_set, topic_nowWp_local;
    
    pn.param<std::string>("service_wpManager_set", service_wpManager_set, "/naviton/waypoint/wpManager/set");
    pn.param<std::string>("topic_nowWp_local", topic_nowWp_local, "/naviton/waypoint/wpManager/nowWp_local");

    _nvt_start_server = nh.advertiseService("/naviton/core/start", &Naviton::start_cb, this);
    _nvt_pause_server = nh.advertiseService("/naviton/core/pause", &Naviton::pause_cb, this);
    _wpManager_set_client = nh.serviceClient<waypoint_manager_msgs::waypoint_manager_set>(service_wpManager_set);
    _nowWp_local_subscriber = nh.subscribe(topic_nowWp_local, 10, &Naviton::nowWp_local_cb, this);
}

void Naviton::init()
{
    waypoint_manager_msgs::waypoint_manager_set srv;
    srv.request.index = 0;
    _wpManager_set_client.call(srv);
    _paused = true;
}

void Naviton::update()
{
    if(_paused) return;

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

bool Naviton::start_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _paused = false;
    return true;
}

bool Naviton::pause_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _paused = true;
    return true;
}