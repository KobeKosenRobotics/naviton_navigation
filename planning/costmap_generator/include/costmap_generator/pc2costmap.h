#ifndef PC_2_COSTMAP_H
#define PC_2_COSTMAP_H

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

class PointCloud2Costmap
{
    public:
        PointCloud2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn);
    private:
        int getIndex(const double x, const double y);
        bool isValidIndex(const int index);  
        void pc_cb(sensor_msgs::PointCloud2::ConstPtr pc_msg);

        ros::Subscriber _pc_sub;
        ros::Publisher _costmap_pub;

        tf2_ros::Buffer _tfBuffer;
        tf2_ros::TransformListener* _tfListener;

        std::string _pc_frameId;
        std::string _robot_frameId;
        float _resolution;
        float _width;
        
        int _grid_width;
        int _grid_num;
        float _width_2;
        int _grid_width_2;
};

#endif