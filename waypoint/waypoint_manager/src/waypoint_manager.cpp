#include "waypoint_manager/waypoint_manager.h"

WaypointManager::WaypointManager(ros::NodeHandle &nh, ros::NodeHandle &pn) : _buffer(), _listener(_buffer)
{
    std::string topic_nowWp, topic_nowWpPose, topic_waypoints, service_manager_set;
    
    pn.param<std::string>("topic_nowWp", topic_nowWp, "wpManager/nowWp");
    pn.param<std::string>("topic_nowWpPose", topic_nowWpPose, "wpManager/nowWpPose");
    pn.param<std::string>("topic_waypoints", topic_waypoints, "wpLoader/waypoints");
    pn.param<std::string>("service_manager_set", service_manager_set, "wpManager/set");

    _nowWp_publisher = nh.advertise<waypoint_msgs::waypoint>(topic_nowWp, 1);
    _nowWpPose_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_nowWpPose, 1);
    _wps_subscriber = nh.subscribe(topic_waypoints, 1, &WaypointManager::waypoints_cb, this);
    _manager_set_server = nh.advertiseService(service_manager_set, &WaypointManager::manager_set_cb, this);

    pn.param<double>("lookupTimeout", _lookupTimeout, 0.2);

    pn.param<std::string>("frame_id_robot", _frame_id_robot, "base_link");
    pn.param<std::string>("frame_id_map", _frame_id_map, "map");

    _index_now = -1;
}

void WaypointManager::publish()
{
    if(_index_now < 0 || _wps.waypoints.size() < _index_now) return;
    _nowWp_publisher.publish(_wps.waypoints[_index_now]);
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = _frame_id_map;
    pose.pose = _wps.waypoints[_index_now].pose;

    try
    {
        _buffer.transform(pose, pose, _frame_id_robot, ros::Duration(_lookupTimeout));
       _nowWpPose_publisher.publish(pose);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
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