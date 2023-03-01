#ifndef NVT_CONTROL__NVT_HWINTERFACE_H
#define NVT_CONTROL__NVT_HWINTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Naviton : public hardware_interface::RobotHW
{
public:
    Naviton();
    void read();
    void write();

private:
    hardware_interface::JointStateInterface _jnt_state_interface;
    hardware_interface::VelocityJointInterface _jnt_vel_interface;
    double _cmd[2];
    double _pos[2];
    double _vel[2];
    double _eff[2];
};

#endif