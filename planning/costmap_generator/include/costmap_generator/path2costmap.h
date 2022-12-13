#ifndef PATH2COSTMAP_H
#define PATH2COSTMAP_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

class Path2Costmap
{
    public:
        Path2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        int get_y_index_from_index(const int index);
        int get_index_from_xy(const double x, const double y);
        bool is_validIndex(const int index);
        void path_cb(const nav_msgs::Path::ConstPtr& path_msg);

        ros::Subscriber _path_sub;
        ros::Publisher _costmap_pub;

        nav_msgs::Path _path;
        nav_msgs::OccupancyGrid _costmap;

        float _width;
        float _resolution;
        int _grid_width;
        int _grid_num;
        double _width_2;
        int _grid_width_2;

        int _cost_min;
        int _cost_max;
        int _lane_width_index_min;
        int _lane_width_index_min2;
        int _lane_width_index_max;
        int _lane_width_index_max2;

        int _lane_width_index_diff;

        bool _path_received;

        int _seq_id;
        std::string _base_link_frame_id;
};

#endif