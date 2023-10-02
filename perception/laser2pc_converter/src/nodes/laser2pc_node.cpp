#include "laser2pc_converter/laser2pc.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser2pc_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    Laser2PointCloud laser2pc(nh, pn);

    while(ros::ok())
    {
        laser2pc.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}