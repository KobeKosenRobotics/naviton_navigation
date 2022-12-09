#ifndef COSTMAP_MERGER_H
#define COSTMAP_MERGER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

class CostmapMerger
{
    public:
        CostmapMerger(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        void costmap_main_cb(const nav_msgs::OccupancyGrid& costmap_msg);
        void costmap_sub_cb(const nav_msgs::OccupancyGrid& costmap_msg);

        int clamp(int val, int min, int max);

        ros::Subscriber _costmap_main_sub;
        ros::Subscriber _costmap_sub_sub;
        ros::Publisher _costmap_merged_pub;

        nav_msgs::OccupancyGrid _costmap_main;
        nav_msgs::OccupancyGrid _costmap_sub;
        nav_msgs::OccupancyGrid _costmap_merged;

        float _costmap_main_magnification;
        float _costmap_sub_magnification;

        float _costmap_main_timeout;
        float _costmap_sub_timeout;

        float _resolution;
        int _grid_width;
        int _grid_num;
        float _width_2;

        int _cost_min;
        int _cost_max;

        int _seq_id;
        std::string _base_link_frame_id;
};

#endif