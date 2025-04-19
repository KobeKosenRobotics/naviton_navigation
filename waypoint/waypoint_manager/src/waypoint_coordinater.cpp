#include "waypoint_manager/waypoint_coordinater.h"

WaypointCoordinater::WaypointCoordinater(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_absolute_position;
    pn.param<std::string>("topic_absolute_position", topic_absolute_position, "/naviton/waypoint//wpManager/fix");  ///naviton/waypoint/wpManager/absolute_position
    abs_wp_publisher = nh.advertise<sensor_msgs::NavSatFix>(topic_absolute_position, 10);

    std::string topic_waypoints, topic_nowWp;
    pn.param<std::string>("topic_waypoints", topic_waypoints, "wpLoader/waypoints");
    pn.param<std::string>("topic_nowWp", topic_nowWp, "/naviton/waypoint/wpManager/nowWp");

    waypoints_sub = nh.subscribe(topic_waypoints, 10, &WaypointCoordinater::waypointsCallback, this);
    _nowWp_subscriber = nh.subscribe(topic_nowWp, 10, &WaypointCoordinater::nowWp_cb, this);

    // YAMLファイルから任意の絶対座標とオリエンテーションを設定
    
    pn.param("absolute_x", absolute_x, 0.0);
    pn.param("absolute_y", absolute_y, 0.0);
    pn.param("absolute_z", absolute_z, 0.0);
    pn.param("absolute_orientation_x", absolute_qx, 0.0);
    pn.param("absolute_orientation_y", absolute_qy, 0.0);
    pn.param("absolute_orientation_z", absolute_qz, 0.0);
    pn.param("absolute_orientation_w", absolute_qw, 1.0);

    tf2::Quaternion abs_quat(absolute_qx, absolute_qy, absolute_qz, absolute_qw);
    _absolute_position.setOrigin(tf2::Vector3(absolute_x, absolute_y, absolute_z));
    _absolute_position.setRotation(abs_quat);

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

geometry_msgs::Pose WaypointCoordinater::calculateRelativePosition(const geometry_msgs::Pose &target, const geometry_msgs::Pose &current)
{
    geometry_msgs::Pose relative_pose;
    relative_pose.position.x = target.position.x - current.position.x;
    relative_pose.position.y = target.position.y - current.position.y;
    relative_pose.position.z = target.position.z - current.position.z;

    tf2::Quaternion q_target, q_current, q_relative;
    // tf2::fromMsg(target.orientation, q_target);
    tf2::fromMsg(current.orientation, q_current);
    q_relative = q_current.inverse() * q_target;
    relative_pose.orientation = tf2::toMsg(q_relative);
    return relative_pose;
}

void WaypointCoordinater::waypointsCallback(const waypoint_msgs::waypoints::ConstPtr &msg)
{
    _wps = *msg;
    // ROS_INFO("Waypoints received: %zu waypoints", _wps.waypoints.size());
}

void WaypointCoordinater::nowWp_cb(waypoint_msgs::waypointConstPtr msg)
{
    _nowWp = *msg;
}

void WaypointCoordinater::publish()
{
    // geometry_msgs::Pose current_wp_pose;
    // try
    // {
    //     geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform("naviton/base_link", "map", ros::Time(0), ros::Duration(1.0));
    //     current_wp_pose.position.x = transform.transform.translation.x;
    //     current_wp_pose.position.y = transform.transform.translation.y;
    //     current_wp_pose.position.z = transform.transform.translation.z;
    //     current_wp_pose.orientation = transform.transform.rotation;
    // }
    // catch (tf2::TransformException &e)
    // {
    //     ROS_WARN("Transform failed: %s", e.what());
    //     return;absolute_x
    // }

    // if (_wp.waypoints.empty())
    // {
    //     ROS_WARN("No waypoints available to publish.");
    //     return;
    // }

    // waypoint_msgs::waypoint waypoint_to_calculate = _wps.waypoints.front();
    // geometry_msgs::Pose relative_pose = calculateRelativePosition(waypoint_to_calculate.pose.pose, current_wp_pose);
    geometry_msgs::Point point = _nowWp.pose.pose.position;

    sensor_msgs::NavSatFix absolute_pose;
    // tf2::Vector3 relative_position(point.y / 133815, point.x / 110000, point.z);
    // tf2::Vector3 absolute_position = _absolute_position.getOrigin() + relative_position;

    // ROS_INFO("Waypoints received: %zu ",absolute_position.x);

    // absolute_pose.latitude = absolute_position.x();
    // absolute_pose.longitude = absolute_position.y();
    // absolute_pose.altitude = absolute_position.z();

    absolute_pose.latitude = 34.7055543 + (point.y / 133815);
    absolute_pose.longitude = 135.5004355 + (point.x / 110000);
    absolute_pose.altitude = 0.0;



    // tf2::Quaternion relative_quat;
    // tf2::fromMsg(relative_pose.orientation, relative_quat);
    // tf2::Quaternion final_orientation = _absolute_position.getRotation() * relative_quat;
    // absolute_pose.pose.orientation = tf2::toMsg(final_orientation);

    abs_wp_publisher.publish(absolute_pose);
}
