#include <ros/ros.h>
#include "nvt_control/teensy_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teensy_handler_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 20);
    ros::Rate loop_rate(frequency);
    
    TeensyHandler handler(nh, pn);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}