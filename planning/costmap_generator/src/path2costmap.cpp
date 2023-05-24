#include "costmap_generator/path2costmap.h"

Path2Costmap::Path2Costmap(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    pn.param<std::string>("base_link_frame_id", _base_link_frame_id, "base_link");

    std::string topic_sub, topic_pub;
    
    pn.param<std::string>("local_path_topic", topic_sub, "local_path_topic");
    pn.param<std::string>("costmap_topic", topic_pub, "costmap_topic");

    _path_sub = nh.subscribe(topic_sub, 10, &Path2Costmap::path_cb, this);
    _costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_pub, 1);

    pn.param<float>("width", _width, 20.0);
    pn.param<float>("resolution", _resolution, 0.1);
    _grid_width = _width / _resolution;
    _grid_num = _grid_width * _grid_width;
    _width_2 = _width / 2.0;
    _grid_width_2 = _grid_width / 2.0;

    _costmap.data.resize(_grid_num);

    pn.param<int>("cost_min", _cost_min, 1);
    pn.param<int>("cost_max", _cost_max, 100);

    float lane_width_min, lane_width_max;
    pn.param<float>("lane_width_min", lane_width_min, 0.0);
    pn.param<float>("lane_width_max", lane_width_max, 5.0);
    _lane_width_index_min = lane_width_min/_resolution*0.5f;
    _lane_width_index_max = lane_width_max/_resolution*0.5f;
    _lane_width_index_min2 = _lane_width_index_min*_lane_width_index_min;
    _lane_width_index_max2 = _lane_width_index_max*_lane_width_index_max;

    _lane_width_index_diff = _lane_width_index_max - _lane_width_index_min;

    pn.param<float>("path_interval_max", _path_interval_max, 1.0);
    _path_interval_max2 = _path_interval_max*_path_interval_max;

    _seq_id = 0;
}

int Path2Costmap::get_y_index_from_index(const int index)
{
    return index / _grid_width;
}

int Path2Costmap::get_index_from_xy(const double x, const double y)
{
    const int _x = floor(x / _resolution + 0.5) + _grid_width_2;
    const int _y = floor(y / _resolution + 0.5) + _grid_width_2;
    return _y * _grid_width + _x;
}

bool Path2Costmap::is_validIndex(const int index)
{
    if(index < 0 || _grid_num <= index) return false;
    return true;
}

bool Path2Costmap::is_validPoint(double x, double y)
{
    if(x <= -_width_2 || x >= _width_2 || y <= -_width_2 || y >= _width_2) return false;
    const int index = get_index_from_xy(x, y);
    return is_validIndex(index);
} // is_validPoint()

void Path2Costmap::input2costmap(const geometry_msgs::Pose pose)
{
    const auto& point = pose.position;
    if(!is_validPoint(point.x, point.y)) return;
    const int index = get_index_from_xy(point.x, point.y);
    _costmap.data[index] = _cost_min;
    for(int i = -_lane_width_index_max; i < _lane_width_index_max; i++)
    {
        for(int j = -_lane_width_index_max; j < _lane_width_index_max; j++)
        {
            const int dist2 = i*i+j*j;
            if(dist2 > _lane_width_index_max2) continue;

            const int index_ = index + i*_grid_width + j;
            if(!is_validIndex(index_) || get_y_index_from_index(index) != get_y_index_from_index(index + j)) continue;

            if(dist2 < _lane_width_index_min2){
                _costmap.data[index_] = _cost_min;
                continue;
            }

            const float t = (sqrt(dist2) - _lane_width_index_min)/(float)(_lane_width_index_diff);
            const int cost = _cost_max * t + _cost_min * (1.0 - t);
            if(cost < _costmap.data[index_]) _costmap.data[index_] = cost;
        }
    }
}

void Path2Costmap::update()
{
    if(!_path_received) return;
    _path_received = true;

    for(int i = 0; i < _grid_num; i++) _costmap.data[i] = _cost_max;

    std::vector<geometry_msgs::Pose> path_interpolate;
    for(int i = 1; i < _path.poses.size(); i++)
    {
        const auto& pos = _path.poses[i].pose.position;
        const auto& pos_old = _path.poses[i - 1].pose.position;
        const float x_diff = pos.x - pos_old.x;
        const float y_diff = pos.y - pos_old.y;
        const float z_diff = pos.z - pos_old.z;
        const float dist2 = x_diff*x_diff + y_diff*y_diff + z_diff*z_diff;
        if(dist2 < _path_interval_max2) continue;
        const int num = sqrt(dist2)/_path_interval_max;

        for(int j = 1; j <= num; j++)
        {
            const float t = (float)(j) / (float)(num + 1.0);
            geometry_msgs::Pose p;
            p.position.x = (1 - t)*pos_old.x + t*pos.x;
            p.position.y = (1 - t)*pos_old.y + t*pos.y;
            p.position.z = (1 - t)*pos_old.z + t*pos.z;
            p.orientation.w = 1.0;
            path_interpolate.push_back(p);
        }
    }

    for(int i = 0; i < _path.poses.size(); i++)
    {
        input2costmap(_path.poses[i].pose);
    }

    for(int i = 0; i < path_interpolate.size(); i++)
    {
        input2costmap(path_interpolate[i]);
    }

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
}

void Path2Costmap::path_cb(const nav_msgs::Path::ConstPtr& path_msg)
{
    if(path_msg->header.frame_id != _base_link_frame_id)
    {
        ROS_WARN("Path2Costmap_node : Frame id does not match");
    }
    _path = *path_msg;
    _path_received = true;
}