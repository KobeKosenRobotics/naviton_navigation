#include "laser2pc_converter/laser2pc.h"

Laser2PointCloud::Laser2PointCloud(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_sub, topic_pub;
    pn.param<std::string>("laser_topic", topic_sub, "laser_topic");
    pn.param<std::string>("pc_topic", topic_pub, "pc_topic");
    _laser_sub = nh.subscribe(topic_sub, 10, &Laser2PointCloud::laser_cb, this);
    _pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_pub, 1);
}

void Laser2PointCloud::laser_cb(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    sensor_msgs::PointCloud2 cloud;
    _projector.projectLaser(*laser_msg, cloud);
    _pc_pub.publish(cloud);
}