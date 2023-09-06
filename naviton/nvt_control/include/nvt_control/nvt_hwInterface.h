#ifndef NVT_CONTROL__NVT_HWINTERFACE_H
#define NVT_CONTROL__NVT_HWINTERFACE_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Naviton : public hardware_interface::RobotHW
{
public:
    Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn);
    void read();
    void write();
    void teensyData_cb(std_msgs::Float32MultiArrayConstPtr msg);

private:
    ros::Subscriber _teensyData_sub;
    tf2_ros::TransformBroadcaster _br;

    hardware_interface::JointStateInterface _jnt_state_interface;
    hardware_interface::VelocityJointInterface _jnt_vel_interface;
    double _cmd[2];
    double _pos[2];
    double _vel[2];
    double _eff[2];

    std::string _frame_id_odom;
    std::string _frame_id_base_link;
};

#endif