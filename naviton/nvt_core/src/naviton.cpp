#include "nvt_core/naviton.h"

Naviton::Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string service_wpManager_set, topic_nowWp_local;
    
    pn.param<std::string>("service_wpManager_set", service_wpManager_set, "/naviton/waypoint/wpManager/set");
    pn.param<std::string>("topic_nowWp_local", topic_nowWp_local, "/naviton/waypoint/wpManager/nowWp_local");
    pn.param<double>("next_wp_distance", _next_wp_distance, 2.0);
    pn.param<double>("next_stop_wp_distance", _next_stop_wp_distance, 0.5);

    _nvt_start_server = nh.advertiseService("/naviton/core/start", &Naviton::start_cb, this);
    _nvt_pause_server = nh.advertiseService("/naviton/core/pause", &Naviton::pause_cb, this);
    _set_pause_client = nh.serviceClient<std_srvs::SetBool>("/safety_planner/set_pause");
    _set_follow_client = nh.serviceClient<std_srvs::SetBool>("/dwa_planner/set_follow");
    _set_safety_client = nh.serviceClient<std_srvs::SetBool>("/safety_planner/set_safety");
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
    double distance_sqr = point.x*point.x+point.y*point.y;
    if(distance_sqr < _next_wp_distance * _next_wp_distance)
    {
        waypoint_manager_msgs::waypoint_manager_set srv;
        if(_nowWp_local.attributes.empty())
        {
            ROS_ERROR("No waypoint attribute.");
            return;
        }
        switch(static_cast<int>(std::round(_nowWp_local.attributes.at(0).type)))
        {
            case waypoint_msgs::waypoint_attribute::TYPE_NEXT_WAYPOINT:
                srv.request.index = _nowWp_local.index + 1;
                _wpManager_set_client.call(srv);
                //ROS_INFO("Next waypoint %d", (int)std::round(_nowWp_local.attributes.at(0).value));
                {
                    std_srvs::SetBool srv;
                    srv.request.data = false;
                    _set_follow_client.call(srv);
                    _set_safety_client.call(srv);
                }
                break;
            case waypoint_msgs::waypoint_attribute::TYPE_SKIP:
                srv.request.index = std::round(_nowWp_local.attributes.at(0).value);
                _wpManager_set_client.call(srv);
                //ROS_INFO("Skip to waypoint %d", (int)std::round(_nowWp_local.attributes.at(0).value));
                break;
            case waypoint_msgs::waypoint_attribute::TYPE_PAUSE:
            {   if(distance_sqr > _next_stop_wp_distance * _next_stop_wp_distance){
                    break;
                }
                //ROS_INFO_STREAM("Pause at waypoint " << _nowWp_local.index);
                std_srvs::SetBool srv;
                srv.request.data = true;
                _set_pause_client.call(srv);
                _paused = true;
                break;
            }
            case waypoint_msgs::waypoint_attribute::TYPE_WP_FOLLOW:
                srv.request.index = _nowWp_local.index + 1;
                _wpManager_set_client.call(srv);
                //ROS_INFO("Follow waypoints from %d", (int)std::round(_nowWp_local.attributes.at(0).value));
                {
                    std_srvs::SetBool srv;
                    srv.request.data = true;
                    _set_follow_client.call(srv);
                    _set_safety_client.call(srv);
                }
                break;
            default:
                break;
        }
    }
}

void Naviton::nowWp_local_cb(waypoint_msgs::waypointConstPtr msg)
{
    _nowWp_local = *msg;
}

bool Naviton::start_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    if(static_cast<int>(std::round(_nowWp_local.attributes.at(0).type)) == waypoint_msgs::waypoint_attribute::TYPE_PAUSE)
    {
        waypoint_manager_msgs::waypoint_manager_set srv;
        srv.request.index = _nowWp_local.index + 1;
        _nowWp_local.attributes.at(0).type = waypoint_msgs::waypoint_attribute::TYPE_NEXT_WAYPOINT;
        _wpManager_set_client.call(srv);
        std_srvs::SetBool bool_srv;
        bool_srv.request.data = false;
        _set_follow_client.call(bool_srv);
        _set_safety_client.call(bool_srv);
    }
    {
        std_srvs::SetBool srv;
        srv.request.data = false;
        _set_pause_client.call(srv);
    }
    
    _paused = false;
    ROS_INFO_STREAM("Naviton started.");
    return true;
}

bool Naviton::pause_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    {
        std_srvs::SetBool srv;
        srv.request.data = true;
        _set_pause_client.call(srv);
    }
    _paused = true;
    ROS_INFO_STREAM("Naviton paused.");
    return true;
}