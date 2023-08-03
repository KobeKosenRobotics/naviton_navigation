#include "costmap_generator/pc2costmap.h"

PointCloud2Costmap::PointCloud2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    tf2_listener = new tf2_ros::TransformListener(tfBuffer);

    pn.param<std::string>("base_link_frame_id", _base_link_frame_id, "base_link");

    std::string topic_sub, topic_sub2, topic_pub;

    pn.param<std::string>("pc_main_topic", topic_sub, "pc_main_topic");
    pn.param<std::string>("pc_sub_topic", topic_sub2, "pc_sub_topic");
    pn.param<std::string>("costmap_topic", topic_pub, "costmap_topic");

    pn.param<bool>("use_pc_sub", _use_pc_sub, true);

    _pc_main_sub = nh.subscribe(topic_sub, 10, &PointCloud2Costmap::pc_main_cb, this);
    
    if(_use_pc_sub)
        _pc_sub_sub = nh.subscribe(topic_sub2, 10, &PointCloud2Costmap::pc_sub_cb, this);
    
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 10);

    pn.param<float>("width", _width, 20.0);
    pn.param<float>("resolution", _resolution, 0.1);
    _grid_width = _width / _resolution;
    _grid_num = _grid_width * _grid_width;
    _width_2 = _width / 2.0;
    _grid_width_2 = _grid_width / 2.0;

    _costmap.data.resize(_grid_num);
    for(int i = 0; i < _grid_num; i++) _costmap.data[i] = _cost_min;

    pn.param<bool>("pc_main_cropping", _pc_main_settings.cropping, "false");
    pn.param<float>("pc_main_downsampling_rate", _pc_main_settings.downsampling_rate, -0.1);
    pn.param<float>("pc_main_height_min", _pc_main_settings.height_min, 0.0);
    pn.param<float>("pc_main_height_max", _pc_main_settings.height_max, 1.0);

    pn.param<bool>("pc_sub_cropping", _pc_sub_settings.cropping, "false");
    pn.param<float>("pc_sub_downsampling_rate", _pc_sub_settings.downsampling_rate, -0.1);
    pn.param<float>("pc_sub_height_min", _pc_sub_settings.height_min, 0.0);
    pn.param<float>("pc_sub_height_max", _pc_sub_settings.height_max, 1.0);

    pn.param<int>("cost_min", _cost_min, 1);
    pn.param<int>("cost_max", _cost_max, 100);

    float inflation_radius;
    pn.param<float>("costmap_inflation_radius", inflation_radius, 0.0);
    _inflation_index = inflation_radius/_resolution;
    _inflation_index2 = _inflation_index*_inflation_index;

    _seq_id = 0;
}

void PointCloud2Costmap::update()
{
    if(!_pc_main_received) return;
    _pc_main_received = false;

    _costmap.header.seq = _seq_id;
    _costmap.header.stamp = ros::Time::now();
    _costmap.header.frame_id = _base_link_frame_id;
    _costmap.info.resolution = _resolution;
    _costmap.info.width = _grid_width;
    _costmap.info.height = _grid_width;
    _costmap.info.origin.position.x = -_width_2;
    _costmap.info.origin.position.y = -_width_2;
    _costmap.info.origin.orientation.w = 1.0;
    _costmap_pub.publish(_costmap);
    _seq_id++;

    for(int i = 0; i < _grid_num; i++) _costmap.data[i] = _cost_min;
}

void PointCloud2Costmap::cropping(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::CropBox<pcl::PointXYZ> crop_box;

    crop_box.setInputCloud(pc_input);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*pc_output);
}

void PointCloud2Costmap::downsampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(pc_input);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*pc_output); 
}

int PointCloud2Costmap::get_x_index_from_index(const int index){
    return index % _grid_width;
}

int PointCloud2Costmap::get_y_index_from_index(const int index){
    return index / _grid_width;
}

int PointCloud2Costmap::get_index_from_xy(const double x, const double y)
{
    const int _x = floor(x / _resolution + 0.5) + _grid_width_2;
    const int _y = floor(y / _resolution + 0.5) + _grid_width_2;
    return _y * _grid_width + _x;
}

bool PointCloud2Costmap::is_validPoint(const double x, const double y)
{
    if(x < -_width_2 || x > _width_2 || y < -_width_2 || y > _width_2) return false;
    const int index = get_index_from_xy(x, y);
    return is_validIndex(index);
}

bool PointCloud2Costmap::is_validIndex(const int index)
{
    if(index < 0 || _grid_num <= index) return false;
    return true;
}

void PointCloud2Costmap::input2costmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
{
    const int cloud_size = pc->points.size();
    if(cloud_size == 0) return;
    
    for(int i = 0; i < cloud_size; i++){

        const auto& p = pc->points[i];
        if(!is_validPoint(p.x, p.y)) continue;
        
        const int index = get_index_from_xy(p.x, p.y);
        if(!is_validIndex(index)) continue;

        if(_costmap.data[get_index_from_xy(p.x, p.y)] < _cost_max)
            _costmap.data[get_index_from_xy(p.x, p.y)] = _cost_max;
        
        for(int j = -_inflation_index; j <= _inflation_index; j++)
        {
            for(int k = -_inflation_index; k <= _inflation_index; k++)
            {
                const int dist2 = j*j+k*k;
                if(dist2 > _inflation_index2) continue;

                const int index_ = index + j*_grid_width + k;
                if(!is_validIndex(index_)) continue;
                
                const int cost = _cost_max*(1.0 - sqrt(dist2)/_inflation_index);
                //const int cost = _cost_max/exp((float)dist2*_resolution*_resolution);
                if(_costmap.data[index_] < cost)
                    _costmap.data[index_] = cost;
            }
        }
    }
}

void PointCloud2Costmap::pc_main_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

    if(pc_msg->header.frame_id != _base_link_frame_id)
    {
        geometry_msgs::TransformStamped transform_stamped = tfBuffer.lookupTransform(_base_link_frame_id, pc_msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        sensor_msgs::PointCloud2 pc_msg_conv;
        Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
        pcl_ros::transformPointCloud(mat, *pc_msg, pc_msg_conv);
        pcl::fromROSMsg(pc_msg_conv, *pc);
    }
    else
    {
        pcl::fromROSMsg(*pc_msg, *pc);
    }
    
    if (pc->points.size() == 0)
		return;

    if(_pc_main_settings.cropping)
    {
        cropping(pc, pc, 
                    Eigen::Vector4f(-_width_2, -_width_2, _pc_main_settings.height_min, 1), 
                    Eigen::Vector4f(_width_2, _width_2, _pc_main_settings.height_max, 1));
    }
    if(_pc_main_settings.downsampling_rate > 0.0)
    {
        downsampling(pc, pc, _pc_main_settings.downsampling_rate);
    }

    input2costmap(pc);

    _pc_main_received = true;
}

void PointCloud2Costmap::pc_sub_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

    if(pc_msg->header.frame_id != _base_link_frame_id)
    {

        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tfBuffer.lookupTransform(_base_link_frame_id, pc_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }
        sensor_msgs::PointCloud2 pc_msg_conv;
        Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
        pcl_ros::transformPointCloud(mat, *pc_msg, pc_msg_conv);
        pcl::fromROSMsg(pc_msg_conv, *pc);
    }
    else
    {
        pcl::fromROSMsg(*pc_msg, *pc);
    }
    
    if (pc->points.size() == 0)
		return;

    if(_pc_sub_settings.cropping)
    {
        cropping(pc, pc, 
                    Eigen::Vector4f(-_width_2, -_width_2, _pc_sub_settings.height_min, 1), 
                    Eigen::Vector4f(_width_2, _width_2, _pc_sub_settings.height_max, 1));
    }
    if(_pc_sub_settings.downsampling_rate > 0.0)
    {
        downsampling(pc, pc, _pc_sub_settings.downsampling_rate);
    }

    input2costmap(pc);
}