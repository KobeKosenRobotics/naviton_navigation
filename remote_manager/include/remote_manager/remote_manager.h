#ifndef REMOTE_MANAGER_H
#define REMOTE_MANAGER_H

#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

class RemoteManager
{
    public:
        RemoteManager(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void init();
        void update();

        void Drive(const geometry_msgs::Twist& cmd_vel_used);
        void Stop();
        void Pause();
        // void Accelerate();
        // void DecelerateStop();

        void cmd_vel_ros_cb(const geometry_msgs::Twist& ros_msg);
        void cmd_vel_remote_cb(const geometry_msgs::Twist& remote_msg);
        void emg_switch_cb(const std_msgs::Bool& emg_msg);
        void mode_switch_cb(const std_msgs::Int32& mode_msg);

    private:
        ros::Subscriber _cmd_vel_ros_sub;
        ros::Subscriber _cmd_vel_remote_sub;
        ros::Subscriber _emg_switch_sub;
        ros::Subscriber _mode_switch_sub;
        ros::Publisher _cmd_vel_pub;

        geometry_msgs::Twist _cmd_vel_ros;
        geometry_msgs::Twist _cmd_vel_remote;
        bool _emg_switch;
        int _mode_switch;
        geometry_msgs::Twist _cmd_vel;

        const geometry_msgs::Twist cmd_vel_zero;

        bool _cmd_vel_ros_updated;
        bool _cmd_vel_remote_updated;
        bool _emg_switch_updated;
        bool _mode_switch_updated;
};

#endif