#include "costmap_generator/costmap_merger.h"

CostmapMerger::CostmapMerger(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    pn.param<std::string>("base_link_frame_id", _base_link_frame_id, "base_link");

    std::string topic_main_sub, topic_sub_sub, topic_pub;
    
    pn.param<std::string>("costmap_main_topic", topic_main_sub, "costmap_main_topic");
    pn.param<std::string>("costmap_sub_topic", topic_sub_sub, "costmap_sub_topic");
    pn.param<std::string>("costmap_merged_topic", topic_pub, "costmap_merged_topic");

    _costmap_main_sub = nh.subscribe(topic_main_sub, 10, &CostmapMerger::costmap_main_cb, this);
    _costmap_sub_sub = nh.subscribe(topic_sub_sub, 10, &CostmapMerger::costmap_sub_cb, this);
    _costmap_merged_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 1);

    pn.param<float>("costmap_main_magnification", _costmap_main_magnification, 1.0);
    pn.param<float>("costmap_sub_magnification", _costmap_sub_magnification, 1.0);

    pn.param<float>("costmap_main_timeout", _costmap_main_timeout, 1.0);
    pn.param<float>("costmap_sub_timeout", _costmap_sub_timeout, 1.0);

    float width;
    pn.param<float>("width", width, 20.0);
    pn.param<float>("resolution", _resolution, 0.1);
    _grid_width = width / _resolution;
    _grid_num = _grid_width * _grid_width;
    _width_2 = width / 2;

    _costmap_main.data.resize(_grid_num);
    _costmap_sub.data.resize(_grid_num);
    _costmap_merged.data.resize(_grid_num);

    pn.param<int>("cost_min", _cost_min, 1);
    pn.param<int>("cost_max", _cost_max, 100);

    _seq_id = 0;
}

void CostmapMerger::update()
{
    const ros::Time time_stamp = ros::Time::now();
    bool input_main = ((time_stamp - _costmap_main.header.stamp).toSec() < _costmap_main_timeout);
    bool input_sub = ((time_stamp - _costmap_sub.header.stamp).toSec() < _costmap_sub_timeout);

    for(int i = 0; i < _grid_num; i++)
    {
        const int cost_main = input_main ? _costmap_main.data[i] : _cost_min;
        const int cost_sub = input_sub ? _costmap_sub.data[i] : _cost_min;
        const int cost = std::max(cost_main, cost_sub);
        _costmap_merged.data[i] = clamp(cost, _cost_min, _cost_max);
    }

    _costmap_merged.header.seq = _seq_id;
    _costmap_merged.header.stamp = ros::Time::now();
    _costmap_merged.header.frame_id = _base_link_frame_id;
    _costmap_merged.info.resolution = _resolution;
    _costmap_merged.info.width = _grid_width;
    _costmap_merged.info.height = _grid_width;
    _costmap_merged.info.origin.position.x = -_width_2;
    _costmap_merged.info.origin.position.y = -_width_2;
    _costmap_merged.info.origin.orientation.w = 1.0;
    _costmap_merged_pub.publish(_costmap_merged);
    _seq_id++;
}

int CostmapMerger::clamp(int val, int min, int max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

void CostmapMerger::costmap_main_cb(const nav_msgs::OccupancyGrid& costmap_msg)
{
    _costmap_main = costmap_msg;
}

void CostmapMerger::costmap_sub_cb(const nav_msgs::OccupancyGrid& costmap_msg)
{
    _costmap_sub = costmap_msg;
}