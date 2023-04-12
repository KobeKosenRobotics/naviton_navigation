#include "costmap_generator/costmap_merger.h"

CostmapMerger::CostmapMerger(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_1_sub, topic_2_sub, topic_pub;
    pn.param<std::string>("costmap_1_topic", topic_1_sub, "costmap_1_topic");
    pn.param<std::string>("costmap_2_topic", topic_2_sub, "costmap_2_topic");
    pn.param<std::string>("costmap_merged_topic", topic_pub, "costmap_merged_topic");

    _costmap_1_sub = nh.subscribe(topic_1_sub, 10, &CostmapMerger::costmap_1_cb, this);
    _costmap_2_sub = nh.subscribe(topic_2_sub, 10, &CostmapMerger::costmap_2_cb, this);
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 1);
}

void CostmapMerger::costmap_1_cb(nav_msgs::OccupancyGrid::ConstPtr costmap_msg)
{
    _og_1 = *costmap_msg;
}

void CostmapMerger::costmap_2_cb(nav_msgs::OccupancyGrid::ConstPtr costmap_msg)
{
    _og_2 = *costmap_msg;
}

void CostmapMerger::update()
{
    nav_msgs::OccupancyGrid og;

    og.header.stamp = ros::Time::now();
    og.header.seq = 0;
    og.header.frame_id = "base_link";

    og.info = _og_1.info;

    const int grid_num_ = _og_1.data.size();
    og.data.resize(grid_num_);

    for(int index = 0; index < grid_num_; index++)
    {
        og.data[index] = std::max(_og_1.data[index], _og_2.data[index]);
    }

    _costmap_pub.publish(og);
}