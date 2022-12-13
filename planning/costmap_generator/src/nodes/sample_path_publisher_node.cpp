#include <ros/ros.h>
#include <nav_msgs/Path.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_path_publisher_node");
    ros::NodeHandle nh;

    const int frequency = 20;

    ros::Rate loop_rate(frequency);

    ros::Publisher sample_pub = nh.advertise<nav_msgs::Path>("/sample_path", 1);

    int seq_id = 0;
    while(ros::ok())
    {
        nav_msgs::Path path;
        path.header.frame_id = "base_link";
        path.header.seq = seq_id;
        path.header.stamp = ros::Time::now();
        seq_id++;
        path.poses.resize(10);
        for(int i = 0; i < 10; i++)
        {
            auto &pose = path.poses[i];
            pose.header.frame_id = path.header.frame_id;
            pose.header.seq = path.header.seq;
            pose.header.stamp = path.header.stamp;
            pose.pose.position.x = i;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
        }
        sample_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}