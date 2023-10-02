#include "nvt_dwa_planner/dwa_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 20);

    ros::Rate loop_rate(frequency);

    DWAPlanner planner(nh, pn);
    
    while(ros::ok())
    {
        planner.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}