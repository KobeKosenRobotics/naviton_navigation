#include "nvt_core/naviton.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "naviton_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 1);

    ros::Rate loop_rate(frequency);

    Naviton nvt(nh, pn);
    nvt.Init();

    while(ros::ok())
    {
        nvt.Update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}