#include "nvt_control/teensy_handler.h"

TeensyHandler::TeensyHandler(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_sub, topic_odom;
    pn.param<std::string>("teensyData_topic", topic_sub, "teensy_data");
    pn.param<std::string>("odom_topic", topic_odom, "mobile_base_controller/odom");

    _teensyData_sub = nh.subscribe(topic_sub, 10, &TeensyHandler::teensyData_cb, this);
    _odom_pub = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);

    pn.param<std::string>("frame_id_odom", _frame_id_odom, "odom");
    pn.param<std::string>("frame_id_base_link", _frame_id_base_link, "base_link");
}

void TeensyHandler::teensyData_cb(std_msgs::Float32MultiArrayConstPtr msg)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    
    transformStamped.header.frame_id = _frame_id_odom;
    transformStamped.child_frame_id =  _frame_id_base_link;

    transformStamped.transform.translation.x = msg->data[0];
    transformStamped.transform.translation.y = msg->data[1];
    transformStamped.transform.translation.z = msg->data[2];

    transformStamped.transform.rotation.w = msg->data[3];
    transformStamped.transform.rotation.x = msg->data[4];
    transformStamped.transform.rotation.y = msg->data[5];
    transformStamped.transform.rotation.z = msg->data[6];

    _br.sendTransform(transformStamped);

    nav_msgs::Odometry odom;
    odom.twist.twist.linear.x = msg->data[7];
    odom.twist.twist.angular.z = msg->data[8];
    _odom_pub.publish(odom);
}