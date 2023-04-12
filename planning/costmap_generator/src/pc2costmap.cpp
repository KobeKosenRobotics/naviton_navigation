#include "costmap_generator/pc2costmap.h"

PointCloud2Costmap::PointCloud2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    _tfListener = new tf2_ros::TransformListener(_tfBuffer);

    std::string topic_sub, topic_pub;
    pn.param<std::string>("pc_topic", topic_sub, "pc_topic");
    pn.param<std::string>("costmap_topic", topic_pub, "costmap_topic");

    pn.param<std::string>("pc_frameId", _pc_frameId, "pc_frameId");
    pn.param<std::string>("robot_frameId", _robot_frameId, "robot_frameId");
    pn.param<float>("resolution", _resolution,0.1);
    pn.param<float>("width", _width, 10.0);
    

    _grid_width = _width/_resolution;
    _grid_num = _grid_width*_grid_width;
    _width_2 = _width/2;
    _grid_width_2 = _grid_width/2;

    _pc_sub = nh.subscribe(topic_sub, 10, &PointCloud2Costmap::pc_cb, this);
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 1);
}

int PointCloud2Costmap::getIndex(const double x, const double y)
{
    int index_x, index_y;
    index_x = (int)(x/_resolution + 0.5) + _grid_width_2;
    if(index_x < 0 || _grid_width <= index_x) return -1;
    index_y = (int)(y/_resolution + 0.5) + _grid_width_2;
    if(index_y < 0 || _grid_width <= index_y) return -1;
    return (index_x)+_grid_width*(index_y);
}

bool PointCloud2Costmap::isValidIndex(const int index)
{
    if(index < 0) return false;
    if(_grid_num <= index) return false;
    return true;
}

void PointCloud2Costmap::pc_cb(sensor_msgs::PointCloud2::ConstPtr pc_msg)
{
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = _tfBuffer.lookupTransform(_robot_frameId, _pc_frameId, ros::Time(0), ros::Duration(1.0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cout << ex.what() << std::endl;
        return;
    }
    sensor_msgs::PointCloud2 pc_msg_conv;
    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat, *pc_msg, pc_msg_conv);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pc_msg_conv, *pc_input);

    nav_msgs::OccupancyGrid og;

    og.header.stamp = ros::Time::now();
    og.header.seq = 0;
    og.header.frame_id = _robot_frameId;

    og.info.width = _grid_width;
    og.info.height = _grid_width;
    og.info.resolution = _resolution;
    
    og.info.origin.orientation.w = 1.0;
    og.info.origin.position.x = -_width_2;
    og.info.origin.position.y = -_width_2;
    og.info.origin.position.z = 0;

    og.data.resize(_grid_num);

    for(const auto & point : pc_input->points)
    {
        int index_ = getIndex(point.x, point.y);
        if(isValidIndex(index_))
        {
            og.data[index_] = 100;
        }
    }

    _costmap_pub.publish(og);
}