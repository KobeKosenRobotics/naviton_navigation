#ifndef NVT_CONTROL__TEENSY_HANDLER_H
#define NVT_CONTROL__TEENSY_HANDLER_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

class TeensyHandler
{
public:
    TeensyHandler(ros::NodeHandle &nh, ros::NodeHandle &pn);
    void teensyData_cb(std_msgs::Float32MultiArrayConstPtr msg);

private:
    ros::Subscriber _teensyData_sub;
    ros::Publisher _odom_pub;
    
    tf2_ros::TransformBroadcaster _br;

    std::string _frame_id_odom;
    std::string _frame_id_base_link;
};

#endif