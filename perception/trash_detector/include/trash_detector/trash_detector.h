#ifndef TRASH_DETECTOR_H
#define TRASH_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

class TrashDetector
{
    public:
        TrashDetector(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        void pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
        void image_cb(const sensor_msgs::ImageConstPtr& msg);
        void boxes_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
        void cam_info_cb(const sensor_msgs::CameraInfoConstPtr& msg);

        ros::Subscriber _pc_sub;
        ros::Subscriber _image_sub;
        ros::Subscriber _boxes_sub;
        ros::Subscriber _cam_info_sub;
        ros::Publisher _marker_pub;

        sensor_msgs::PointCloud2 _pc_msg;
        sensor_msgs::Image _image_msg;
        darknet_ros_msgs::BoundingBoxes _boxes_msg;
        sensor_msgs::CameraInfo _cam_info_msg;

        bool _pc_subscribed;
        bool _image_subscribed;
        bool _boxes_subscribed;
        bool _cam_info_subscribed;
};

#endif