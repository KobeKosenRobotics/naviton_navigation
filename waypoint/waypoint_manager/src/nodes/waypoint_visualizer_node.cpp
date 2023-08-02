#include "waypoint_manager/waypoint_visualizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    WaypointVisualizer visualizer(nh, pn);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}