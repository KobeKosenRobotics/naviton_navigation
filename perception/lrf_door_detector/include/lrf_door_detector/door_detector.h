#ifndef DOOR_DETECTOR
#define DOOR_DETECTOR

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class DoorDetector
{
    public:
        DoorDetector(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
        void pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    private:
        ros::Subscriber _pc_sub;
        ros::Publisher _pc_pub;

        pcl::PointCloud<pcl::PointXYZ> _pc_input;

        bool _pc_subscribed;
};

#endif