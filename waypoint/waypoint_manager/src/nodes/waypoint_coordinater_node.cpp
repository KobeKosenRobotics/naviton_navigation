#include <ros/ros.h>
#include "waypoint_manager/waypoint_coordinater.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_coordinater_node");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    // WaypointCoordinaterのインスタンス化
    WaypointCoordinater coordinater(nh, pn);

    // パブリッシュループの設定
    ros::Rate loop_rate(10); // 10Hzでパブリッシュ

    while (ros::ok())
    {
        // 絶対座標をパブリッシュ
        coordinater.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
