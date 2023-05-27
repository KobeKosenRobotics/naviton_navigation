#ifndef PC2COSTMAP_H
#define PC2COSTMAP_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

struct PointCloudProcessingSettings
{
    bool cropping;
    float downsampling_rate;
    float height_min;
    float height_max;
};

using PCPS = PointCloudProcessingSettings;

class PointCloud2Costmap
{
    public:
        PointCloud2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void update();
    private:
        void cropping(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        void downsampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, float leaf_size);
        void input2costmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);

        int get_x_index_from_index(const int index);
        int get_y_index_from_index(const int index);
        int get_index_from_xy(const double x, const double y);
        bool is_validPoint(const double x, const double y);
        bool is_validIndex(const int index);

        void pc_main_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
        void pc_sub_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener* tf2_listener;

        ros::Subscriber _pc_main_sub;
        ros::Subscriber _pc_sub_sub;
        ros::Publisher _costmap_pub;

        nav_msgs::OccupancyGrid _costmap;

        bool _use_pc_sub;

        bool _pc_main_received;

        float _width;
        float _resolution;
        int _grid_width;
        int _grid_num;
        double _width_2;
        int _grid_width_2;

        int _cost_min;
        int _cost_max;
        int _inflation_index;
        int _inflation_index2;

        PCPS _pc_main_settings;
        PCPS _pc_sub_settings;

        int _seq_id;
        std::string _base_link_frame_id;
};

#endif