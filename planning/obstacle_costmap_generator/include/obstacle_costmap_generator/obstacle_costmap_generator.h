#ifndef OBSTACLE_COSTMAP_GENERATOR_H
#define OBSTACLE_COSTMAP_GENERATOR_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

class ObstacleCostmapGenerator
{
    public:
        ObstacleCostmapGenerator(ros::NodeHandle& nh, ros::NodeHandle& pn);
        void update();
    private:
        void generate_costmap();
        void cropping(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
        void downsampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, float leaf_size);
        void projection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXY>::Ptr pc_output);
        void input2costmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);
        double lerp(double a, double b, double t);
        void pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

        tf2_ros::Buffer _buffer;
        tf2_ros::TransformListener _listener;

        ros::Subscriber _pc_sub;
        ros::Publisher _costmap_pub;

        nav_msgs::OccupancyGrid _costmap;
        std::vector<pcl::PointXY> _costmap_points;

        float _width;
        float _resolution;
        int _grid_width;
        int _grid_num;
        
        float _width_2;
        float _width_2_inv;
        int _grid_width_2;

        float _height_min;
        float _height_max;

        std::string _base_link_frame_id;

        sensor_msgs::PointCloud2 _pc;
        bool _pc_subscribed;
};

#endif