#include "costmap_generator/path2costmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path2costmap_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    Path2Costmap path2costmap(nh, pn);

    while(ros::ok())
    {
        path2costmap.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}