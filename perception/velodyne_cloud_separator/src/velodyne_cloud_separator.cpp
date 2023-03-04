#include "velodyne_cloud_separator/velodyne_cloud_separator.h"

VelodyneCloudSeparator::VelodyneCloudSeparator(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_sub, topic_ground_pub, topic_obstacle_pub;
    pn.param<std::string>("pc_raw_topic", topic_sub, "/velodyne_points");
    pn.param<std::string>("pc_ground_topic", topic_ground_pub, "/velodyne_points/ground");
    pn.param<std::string>("pc_obstacle_topic", topic_obstacle_pub, "/velodyne_points/obstacle");

    _pc_sub = nh.subscribe(topic_sub, 10, &VelodyneCloudSeparator::pc_cb, this);
    _pc_ground_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_ground_pub, 1);
    _pc_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_obstacle_pub, 1);

    pn.param<float>("sensor_height", _sensor_height, 0.8);

    pn.param<float>("radius_coeff_close", _radius_coeff_close, 0.2);
    pn.param<float>("radius_coeff_far", _radius_coeff_far, 0.7);

    float max_slope;
    pn.param<float>("max_slope", max_slope, 15.0);
    _limiting_ratio = tan(max_slope*M_PI/180.0);

    pn.param<int>("point_num_min", _point_num_min, 2);

    pn.param<float>("gap_threshold", _gap_threshold, 0.15);

    init();

    _seq_id = 0;
}

void VelodyneCloudSeparator::update()
{
    if(_pc_input.header.frame_id == "") return;
    pcl::PointCloud<pcl::PointXYZ> pc_ground;
    pcl::PointCloud<pcl::PointXYZ> pc_obstacle;
    pc_ground.clear();
    pc_obstacle.clear();

    separate(pc_ground, pc_obstacle);

    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(pc_ground, pc_msg);
    pc_msg.header.frame_id = _frame_id;
    pc_msg.header.seq = _seq_id;
    pc_msg.header.stamp = ros::Time::now();
    _pc_ground_pub.publish(pc_msg);

    pcl::toROSMsg(pc_obstacle, pc_msg);
    pc_msg.header.frame_id = _frame_id;
    pc_msg.header.seq = _seq_id;
    pc_msg.header.stamp = ros::Time::now();
    _pc_obstacle_pub.publish(pc_msg);

    _seq_id++;
}

void VelodyneCloudSeparator::init()
{
    float start_angle = 92.0/3.0;
    float angle_res = 4.0/3.0;

    _radius_array[0] = FLT_MAX;
    for(int i = 1; i < _height; i++)
    {
        if(i > 20)
        {
            _radius_array[i] = _radius_array[20];
            continue;
        }
        double theta = start_angle - i * angle_res;
        theta *= M_PI/180.0f;
        _radius_array[i] = _sensor_height*(1.0/tan(theta) - 1.0/tan(theta + angle_res*M_PI/180.0));
        _radius_array[i] *= (i<=12?_radius_coeff_close:_radius_coeff_far);
    }

    for(int i = 0; i < _height; i++) _point_types[i] = UNKNOWN;
}

void VelodyneCloudSeparator::separate(pcl::PointCloud<pcl::PointXYZ>& pc_ground,
                                        pcl::PointCloud<pcl::PointXYZ>& pc_obstacle)
{
    int width = (int)(_pc_input.points.size()*1.8/_height);
    _index_map = cv::Mat_<int>(_height, width, -1);

    for(int i = 0; i < _pc_input.points.size(); i++)
    {
        const auto& pt = _pc_input.points[i];
        float angle = atan2(pt.y, pt.x)*180/M_PI;
        if(angle < 0) angle += 360.0;
        int row = _height - 1 - pt.ring;
        int column = width - (int)((float)width * angle / 360.0) - 1;
        _index_map.at<int>(row, column) = i; 
    }

    for(int i = 0; i < width; i++)
    {
        PointType point_types[_height];

        int point_index[_height];
        int point_index_size = 0;

        int unknown_index[_height];
        int unknown_index_size = 0;

        float z_ref = 0.0;
        float r_ref = 0.0;

        std::copy(_point_types, _point_types + _height, point_types);

        for(int j = _height - 1; j >= 0; j--)
        {
            if(_index_map.at<int>(j, i) > -1 && point_types[j] == UNKNOWN)
            {
                float x0 = _pc_input.points[_index_map.at<int>(j, i)].x;
                float y0 = _pc_input.points[_index_map.at<int>(j, i)].y;
                float z0 = _pc_input.points[_index_map.at<int>(j, i)].z;
                float r0 = sqrt(x0*x0 + y0*y0);
                float r_diff = r0 - r_ref;
                float z_diff = fabs(z0 - z_ref);
                float angle = z_diff / r_diff;

                if((angle > 0 && angle < _limiting_ratio && z_diff < _gap_threshold) || point_index_size == 0)
                {
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
                else
                {
                    for(int k = 0; k < point_index_size; k++)
                    {
                        int index = _index_map.at<int>(point_index[k], i);
                        if(point_index_size > _point_num_min)
                        {
                            pcl::PointXYZ point;
                            point.x = _pc_input.points[index].x;
                            point.y = _pc_input.points[index].y;
                            point.z = _pc_input.points[index].z;
                            //point.intensity = pc_input->points[index].intensity;
                            //point.ring = pc_input->points[index].ring;
                            pc_ground.push_back(point);

                            point_types[point_index[k]] = GROUND;
                        }
                        else
                        {
                            unknown_index[unknown_index_size] = index;
                            unknown_index_size++;
                        }
                    }
                    point_index_size = 0;
                    r_ref = r0;
                    z_ref = z0;
                    point_index[point_index_size] = j;
                    point_index_size++;
                }
            }

            if(j != 0) continue;

            if(point_index != 0)
            {
                for(int k = 0; k < point_index_size; k++)
                {
                    int index = _index_map.at<int>(point_index[k], i);
                    if(point_index_size > _point_num_min)
                    {
                        pcl::PointXYZ point;
                        point.x = _pc_input.points[index].x;
                        point.y = _pc_input.points[index].y;
                        point.z = _pc_input.points[index].z;
                        //point.intensity = pc_input->points[index].intensity;
                        //point.ring = pc_input->points[index].ring;
                        pc_ground.push_back(point);
                        point_types[point_index[k]] = GROUND;
                    }
                    else
                    {
                        unknown_index[unknown_index_size] = index;
                        unknown_index_size++;
                    }
                }
                point_index_size = 0;
            }

            float centroid = 0;
            int centroid_ring = 0;
            int cluster_index[_height];
            int cluster_index_size = 0;
            for(int k = unknown_index_size - 1; k >= 0; k--)
            {
                float x0 = _pc_input.points[unknown_index[k]].x;
                float y0 = _pc_input.points[unknown_index[k]].y;
                float r0 = sqrt(x0*x0+y0*y0);
                float r_diff = fabs(r0 - centroid);
                float dist = _radius_array[centroid_ring];
                if(r_diff >= dist && cluster_index_size != 0)
                {
                    for(int l = 0; l < cluster_index_size; l++)
                    {
                        pcl::PointXYZ point;
                        point.x = _pc_input.points[cluster_index[l]].x;
                        point.y = _pc_input.points[cluster_index[l]].y;
                        point.z = _pc_input.points[cluster_index[l]].z;
                        (cluster_index_size > 1 ? pc_obstacle : pc_ground).push_back(point);
                    }
                    cluster_index_size = 0;
                }
                cluster_index[cluster_index_size] = unknown_index[k];
                cluster_index_size++;
                centroid = r0;
                centroid_ring = _pc_input.points[unknown_index[k]].ring;
                if(k != 0) continue;
                for(int l = 0; l < cluster_index_size; l++)
                {
                    pcl::PointXYZ point;
                    point.x = _pc_input.points[cluster_index[l]].x;
                    point.y = _pc_input.points[cluster_index[l]].y;
                    point.z = _pc_input.points[cluster_index[l]].z;
                    (cluster_index_size > 1 ? pc_obstacle : pc_ground).push_back(point);
                }
                cluster_index_size = 0;
            }
        }
    }
}

void VelodyneCloudSeparator::pc_cb(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
    _frame_id = pc_msg->header.frame_id;
    pcl::fromROSMsg(*pc_msg, _pc_input);
}