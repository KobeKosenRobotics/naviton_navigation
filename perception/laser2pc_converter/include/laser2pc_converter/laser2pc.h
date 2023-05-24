#ifndef LASER_2_PC_H
#define LASER_2_PC_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

class Laser2PointCloud
{
    public:
        Laser2PointCloud(ros::NodeHandle &nh, ros::NodeHandle &pn);
        
    private:
        void laser_cb(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

        ros::Subscriber _laser_sub;
        ros::Publisher _pc_pub;
        laser_geometry::LaserProjection _projector;
};

#endif