#include "a_star_planner/a_star_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 1);

    ros::Rate loop_rate(frequency);

    AStarPlanner planner(nh, pn);

    while(ros::ok())
    {
        planner.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}