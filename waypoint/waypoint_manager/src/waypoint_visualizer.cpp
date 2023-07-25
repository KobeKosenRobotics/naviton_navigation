#include "waypoint_manager/waypoint_visualizer.h"

WaypointVisualizer::WaypointVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_markerArray, topic_wps;
    
    pn.param<std::string>("topic_markerArray", topic_markerArray, "wpVisualizer/markerArray");
    pn.param<std::string>("topic_waypoints", topic_wps, "wpLoader/waypoints");

    _markerArray_publisher = nh.advertise<visualization_msgs::MarkerArray>(topic_markerArray, 1);
    _wps_subscriber = nh.subscribe(topic_wps, 1, &WaypointVisualizer::waypoints_cb, this);

    pn.param<std::string>("frame_id_map", _frame_id_map, "map");
    pn.param<double>("scale", _text_size, 1.0);
    pn.param<double>("z_offset", _z_offset, 0.0);
}

void WaypointVisualizer::waypoints_cb(waypoint_msgs::waypointsConstPtr msg)
{
    visualization_msgs::MarkerArray markerArray;
    
    for(int i = 0; i < msg->waypoints.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = _frame_id_map;
        marker.header.stamp = ros::Time::now();
        marker.id = msg->waypoints[i].index;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; 
        marker.pose = msg->waypoints[i].pose;
        marker.scale.x = _text_size;
        marker.scale.y = _text_size;
        marker.scale.z = _text_size;
        marker.pose.position.z += _z_offset;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.text = std::to_string(i);
        /*
        for(int j = 0; j < msg->waypoints[i].attributes.size() && j < msg->waypoints[i].attribute_values.size(); j++)
        {
            marker.text += "\n" + msg->waypoints[i].attributes[j] + ":" + std::to_string(msg->waypoints[i].attribute_values[j]);
        }
        */
        markerArray.markers.push_back(marker);
    }
    _markerArray_publisher.publish(markerArray);
}