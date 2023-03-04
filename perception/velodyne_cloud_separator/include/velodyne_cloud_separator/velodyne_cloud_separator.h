#ifndef VELODYNE_CLOUD_SEPARATOR_H
#define VELODYNE_CLOUD_SEPARATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <iostream>

struct PointXYZIR{
PCL_ADD_POINT4D;
float intensity;
uint16_t ring;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
(float,x,x)
(float,y,y)
(float,z,z)
(float,intensity,intensity)
(uint16_t,ring,ring)
)

enum PointType
{
    UNKNOWN = 0,
    GROUND = 1,
    OBSTACLE = 2,
};

class VelodyneCloudSeparator
{
    public:
        VelodyneCloudSeparator(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        void init();
        void separate(pcl::PointCloud<pcl::PointXYZ>& pc_ground,
                        pcl::PointCloud<pcl::PointXYZ>& pc_obstacle);

        void pc_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);

        ros::Subscriber _pc_sub;
        pcl::PointCloud<PointXYZIR> _pc_input;

        ros::Publisher _pc_ground_pub;
        ros::Publisher _pc_obstacle_pub;

        cv::Mat _index_map;
        PointType _point_types[32];

        float _sensor_height;

        float _radius_coeff_close;
        float _radius_coeff_far;

        float _radius_array[32];

        float _limiting_ratio;

        int _point_num_min;
        float _gap_threshold;

        int _height = 32;

        std::string _frame_id;
        int _seq_id;
};

#endif