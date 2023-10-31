#include "obstacle_costmap_generator/obstacle_costmap_generator.h"

ObstacleCostmapGenerator::ObstacleCostmapGenerator(ros::NodeHandle& nh, ros::NodeHandle& pn) : _listener(_buffer)
{
    pn.param<std::string>("base_link_frame_id", _base_link_frame_id, "base_link");

    std::string topic_sub, topic_pub;

    pn.param<std::string>("pc_topic", topic_sub, "pc_topic");
    pn.param<std::string>("costmap_topic", topic_pub, "costmap_topic");

    _pc_sub = nh.subscribe(topic_sub, 1, &ObstacleCostmapGenerator::pc_cb, this);
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 10);

    pn.param<float>("width", _width, 20.0);
    pn.param<float>("resolution", _resolution, 0.1);
    _grid_width = _width / _resolution;
    _grid_num = _grid_width * _grid_width;
    _width_2 = _width / 2.0;
    _width_2_inv = 1.0 / _width_2;
    _grid_width_2 = _grid_width / 2.0;

    pn.param<float>("height_min", _height_min, 0.0);
    pn.param<float>("height_max", _height_max, 1.0);

    _costmap.header.frame_id = _base_link_frame_id;
    _costmap.header.seq = 0;
    _costmap.info.resolution = _resolution;
    _costmap.info.width = _grid_width;
    _costmap.info.height = _grid_width;
    _costmap.info.origin.position.x = -_width_2;
    _costmap.info.origin.position.y = -_width_2;
    _costmap.info.origin.orientation.w = 1.0;

    _costmap.data.resize(_grid_num);
    
    _costmap_points.resize(_grid_num);
    for(int y = 0; y < _grid_width; y++)
    {
        int index_y = y * _grid_width;
        for(int x = 0; x < _grid_width; x++)
        {
            int index = index_y + x;
            _costmap_points[index].x = ((double)(x - _grid_width_2) + 0.5) * _resolution;
            _costmap_points[index].y = ((double)(y - _grid_width_2) + 0.5) * _resolution;
        }
    }
}

void ObstacleCostmapGenerator::update()
{
    if(!_pc_subscribed) return;

    std::fill(_costmap.data.begin(), _costmap.data.end(), 1);
    generate_costmap();

    _costmap.header.stamp = ros::Time::now();
    _costmap.header.seq++;

    _costmap_pub.publish(_costmap);

    _pc_subscribed = false;
}

void ObstacleCostmapGenerator::generate_costmap()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    if(_pc.header.frame_id != _base_link_frame_id)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = _buffer.lookupTransform(_base_link_frame_id, _pc.header.frame_id, ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        sensor_msgs::PointCloud2 pc_conv;
        Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
        pcl_ros::transformPointCloud(mat, _pc, pc_conv);
        pcl::fromROSMsg(pc_conv, *pc);
    }
    else
    {
        pcl::fromROSMsg(_pc, *pc);
    }

    if (pc->points.size() == 0) return;

    cropping(pc, pc, 
                    Eigen::Vector4f(-_width_2, -_width_2, _height_min, 1), 
                    Eigen::Vector4f(_width_2, _width_2, _height_max, 1));
    
    if (pc->points.size() == 0) return;
    
    downsampling(pc, pc, _resolution);
    
    input2costmap(pc);
}

void ObstacleCostmapGenerator::cropping(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::CropBox<pcl::PointXYZ> crop_box;

    crop_box.setInputCloud(pc_input);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*pc_output);
}

void ObstacleCostmapGenerator::downsampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_output, float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(pc_input);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*pc_output); 
}

void ObstacleCostmapGenerator::projection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc_input, pcl::PointCloud<pcl::PointXY>::Ptr pc_output)
{
    pc_output->points.resize(pc_input->points.size());
    
    #pragma omp parallel for
    for(int i = 0; i < pc_input->points.size(); i++)
    {
        pc_output->points[i].x = pc_input->points[i].x;
        pc_output->points[i].y = pc_input->points[i].y;
    }
}

void ObstacleCostmapGenerator::input2costmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
{
    pcl::PointCloud<pcl::PointXY>::Ptr pc_2d (new pcl::PointCloud<pcl::PointXY>);
    projection(pc, pc_2d);

    pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXY>);
    tree->setInputCloud(pc_2d);
    
    for(int i = 0; i < _grid_num; i++)
    {
        pcl::PointXY p = _costmap_points[i];
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;

        tree->nearestKSearch(p, 1, k_indices, k_sqr_distances);
        if(k_sqr_distances.size() <= 0) break;

        float distance = sqrt(k_sqr_distances[0]);
        int cost = lerp(100, 1, distance * _width_2_inv);
        if(cost < _costmap.data[i]) continue;
        _costmap.data[i] =  cost; 
    }
}

double ObstacleCostmapGenerator::lerp(double a, double b, double t)
{
    t = std::max(std::min(t, 1.0), 0.0);
    return a * (1.0 - t) + b * t; 
}

void ObstacleCostmapGenerator::pc_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
    _pc = *pc_msg;

    _pc_subscribed = true;
}