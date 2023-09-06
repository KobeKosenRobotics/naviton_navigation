#include "nvt_core/naviton.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "naviton_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 20);

    ros::Rate loop_rate(frequency);

    Naviton nvt(nh, pn);
    nvt.init();

    while(ros::ok())
    {
        nvt.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}