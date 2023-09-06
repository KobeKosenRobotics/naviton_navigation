#include "nvt_control/nvt_hwInterface.h"

Naviton::Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_sub;
    pn.param<std::string>("teensyData_topic", topic_sub, "teensy_data");
    _teensyData_sub = nh.subscribe(topic_sub, 10, &Naviton::teensyData_cb, this);

    pn.param<std::string>("frame_id_odom", _frame_id_odom, "odom");
    pn.param<std::string>("frame_id_base_link", _frame_id_base_link, "base_link");
    
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
    // ROS_INFO_STREAM("Commands for joints: " << _cmd[0] << ", " << _cmd[1]);
}

void Naviton::write()
{

}

void Naviton::teensyData_cb(std_msgs::Float32MultiArrayConstPtr msg)
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
}