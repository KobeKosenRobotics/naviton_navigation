#include "path_costmap_generator/path_costmap_generator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_costmap_generator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 20);

    ros::Rate loop_rate(frequency);

    PathCostmapGenerator generator(nh, pn);
    
    while(ros::ok())
    {
        generator.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}