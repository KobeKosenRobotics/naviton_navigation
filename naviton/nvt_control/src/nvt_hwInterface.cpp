#include "nvt_control/nvt_hwInterface.h"

Naviton::Naviton()
{
    _pos[0] = _pos[1] = 0;
    _vel[0] = _vel[1] = 0;
    _eff[0] = _eff[1] = 0;
    _cmd[0] = _cmd[1] = 0;

    hardware_interface::JointStateHandle state_handle_1("left_wheel_joint", &_pos[0], &_vel[0], &_eff[0]);
    _jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("right_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _jnt_state_interface.registerHandle(state_handle_2);

    registerInterface(&_jnt_state_interface);

    hardware_interface::JointHandle vel_handle_1(_jnt_state_interface.getHandle("left_wheel_joint"), &_cmd[0]);
    _jnt_vel_interface.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(_jnt_state_interface.getHandle("right_wheel_joint"), &_cmd[1]);
    _jnt_vel_interface.registerHandle(vel_handle_2);

    registerInterface(&_jnt_vel_interface);
}

void Naviton::read()
{
    ROS_INFO_STREAM("Commands for joints: " << _cmd[0] << ", " << _cmd[1]);
}

void Naviton::write()
{

}