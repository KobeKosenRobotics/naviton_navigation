#include "costmap_generator/pc2costmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc2costmap_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    PointCloud2Costmap pc2costmap(nh, pn);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}