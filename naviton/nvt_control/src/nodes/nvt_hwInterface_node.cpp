#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "nvt_control/nvt_hwInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nvt_hwInterface_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    Naviton nvt;
    controller_manager::ControllerManager cm(&nvt);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time t = ros::Time::now();
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::Duration d = ros::Time::now() - t;
        ros::Time t = ros::Time::now();
        nvt.read();
        cm.update(t, d);
        nvt.write();
        ros::spinOnce();
        loop_rate.sleep();
    }

    spinner.stop();

    return 0;
}