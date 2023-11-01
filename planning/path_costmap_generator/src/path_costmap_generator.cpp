#include "path_costmap_generator/path_costmap_generator.h"

PathCostmapGenerator::PathCostmapGenerator(ros::NodeHandle &nh, ros::NodeHandle &pn) : _listener(_buffer)
{
    std::string topic_waypoint, topic_waypoints;

    pn.param<std::string>("topic_waypoint", topic_waypoint, "/waypoint_manager/wpManager/nowWp");
    pn.param<std::string>("topic_waypoints", topic_waypoints, "/waypoint_manager/wpLoader/waypoints");
    
    _wp_sub = nh.subscribe(topic_waypoint, 1, &PathCostmapGenerator::waypoint_cb, this);
    _wps_sub = nh.subscribe(topic_waypoints, 1, &PathCostmapGenerator::waypoints_cb, this);
    
    pn.param<float>("width", _width, 20.0);
    pn.param<float>("resolution", _resolution, 0.1);
    _grid_width = _width / _resolution;
    _grid_num = _grid_width * _grid_width;
    _width_2 = _width / 2.0;
    _grid_width_2 = _grid_width / 2.0;

    _wp_subscribed = false;
    _wps_subscribed = false;
}

void PathCostmapGenerator::update()
{
    if(!_wp_subscribed || !_wps_subscribed) return;

    geometry_msgs::TransformStamped map2base;
    try
    {
        map2base = _buffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    int wpIndex_start, wpIndex_end;

    for(wpIndex_start = _wpIndex_now; wpIndex_start > 0; wpIndex_start--)
    {
        geometry_msgs::PoseStamped pose;
        tf2::doTransform(_wps.waypoints[wpIndex_start].pose, pose, map2base);
        double x = pose.pose.position.x;
        double y = pose.pose.position.y; 
        if(x > _width_2 || x < -_width_2 || y > _width_2 || y < -_width_2) break;
    }

    for(wpIndex_end = _wpIndex_now; wpIndex_end < _wps.waypoints.size(); wpIndex_end++)
    {
        geometry_msgs::PoseStamped pose;
        tf2::doTransform(_wps.waypoints[wpIndex_end].pose, pose, map2base);
        double x = pose.pose.position.x;
        double y = pose.pose.position.y; 
        if(x > _width_2 || x < -_width_2 || y > _width_2 || y < -_width_2) break;
    }

    #pragma omp parallel for
    for(int i = 0; i < _grid_num; i++)
    {
        
    }
}

void PathCostmapGenerator::waypoint_cb(waypoint_msgs::waypointConstPtr msg)
{
    _wpIndex_now = msg->index;

    _wp_subscribed = true;
}

void PathCostmapGenerator::waypoints_cb(waypoint_msgs::waypointsConstPtr msg)
{
    _wps = *msg;

    _wps_subscribed = true;
}