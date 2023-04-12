#ifndef COSTMAP_MERGER_H
#define COSTMAP_MERGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

class CostmapMerger
{
    public:
        CostmapMerger(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        // int getIndex(const double x, const double y);
        // bool isValidIndex(const int index);  
        void costmap_1_cb(nav_msgs::OccupancyGrid::ConstPtr costmap_msg);
        void costmap_2_cb(nav_msgs::OccupancyGrid::ConstPtr costmap_msg);

        ros::Subscriber _costmap_1_sub;
        ros::Subscriber _costmap_2_sub;
        ros::Publisher _costmap_pub;

        nav_msgs::OccupancyGrid _og_1;
        nav_msgs::OccupancyGrid _og_2;

        // tf2_ros::Buffer _tfBuffer;
        // tf2_ros::TransformListener* _tfListener;

        // std::string _pc_frameId;
        // std::string _robot_frameId;
        // float _resolution;
        // float _width;
        
        // int _grid_width;
        // int _grid_num;
        // float _width_2;
        // int _grid_width_2;
};

#endif