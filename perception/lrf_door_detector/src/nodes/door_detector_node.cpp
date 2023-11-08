#include "lrf_door_detector/door_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "door_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int frequency;
    pn.param<int>("frequency", frequency, 10);

    ros::Rate loop_rate(frequency);

    DoorDetector detector(nh, pn);

    while(ros::ok())
    {
        detector.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}