#include "waypoint_manager/waypoint_visualizer.h"

WaypointVisualizer::WaypointVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_markerArray, topic_wps_1, topic_wps_2;
    
    pn.param<std::string>("topic_markerArray", topic_markerArray, "wpVisualizer/markerArray");
    pn.param<std::string>("topic_waypoints_1", topic_wps_1, "wpRecorder/waypoints");
    pn.param<std::string>("topic_waypoints_2", topic_wps_2, "wpLoader/waypoints");

    _markerArray_publisher = nh.advertise<visualization_msgs::MarkerArray>(topic_markerArray, 1);
    _wps_subscriber_1 = nh.subscribe(topic_wps_1, 1, &WaypointVisualizer::waypoints_cb, this);
    _wps_subscriber_2 = nh.subscribe(topic_wps_2, 1, &WaypointVisualizer::waypoints_cb, this);

    pn.param<double>("scale", _text_size, 1.0);
    pn.param<double>("z_offset", _z_offset, 0.0);
}

void WaypointVisualizer::waypoints_cb(waypoint_msgs::waypointsConstPtr msg)
{
    visualization_msgs::MarkerArray markerArray;
    
    for(int i = 0; i < msg->waypoints.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id = msg->waypoints[i].index;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; 
        marker.pose = msg->waypoints[i].pose.pose;
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