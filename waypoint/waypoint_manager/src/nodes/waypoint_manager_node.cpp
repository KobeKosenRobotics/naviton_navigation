#include "waypoint_manager/waypoint_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    WaypointManager manager(nh, pn);

    while(ros::ok())
    {
        manager.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}