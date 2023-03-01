#include "velodyne_cloud_separator/velodyne_cloud_separator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_cloud_separator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    VelodyneCloudSeparator vcs(nh, pn);
    
    while(ros::ok())
    {
        vcs.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}