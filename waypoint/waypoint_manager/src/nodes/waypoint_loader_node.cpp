#include "waypoint_manager/waypoint_loader.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 1);

    std::string file_dir;
    pn.param<std::string>("file_dir", file_dir, "/home");

    std::string file_name;
    pn.param<std::string>("file_name", file_name, "wpData_latest.csv");

    ros::Rate loop_rate(frequency);

    WaypointLoader loader(nh, pn);
    loader.load(file_dir + "/" + file_name);

    while(ros::ok())
    {
        ros::spinOnce();
        loader.publish();
        loop_rate.sleep();
    }

    return 0;
}