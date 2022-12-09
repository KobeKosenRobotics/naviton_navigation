#include "costmap_generator/path2costmap.h"

Path2Costmap::Path2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_sub, topic_pub;
    
    pn.param<std::string>("path_topic", topic_sub, "path_topic");
    pn.param<std::string>("costmap_topic", topic_pub, "costmap_topic");

    _path_sub = nh.subscribe(topic_sub, 10, &Path2Costmap::path_cb, this);
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 1);
}

void Path2Costmap::update()
{
}

void Path2Costmap::path_cb(const nav_msgs::Path::ConstPtr& path_msg)
{
}