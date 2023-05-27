#include "costmap_generator/costmap_merger.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_merger_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    CostmapMerger costmap_merger(nh, pn);

    while(ros::ok())
    {
        costmap_merger.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}