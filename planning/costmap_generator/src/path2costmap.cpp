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

void Path2Costmap::update()
{
    if(!_path_received) return;
    _path_received = true;

    for(int i = 0; i < _grid_num; i++) _costmap.data[i] = _cost_max;


    for(int i = 0; i < _path.poses.size(); i++)
    {
        const auto& point = _path.poses[i].pose.position;
        const int index = get_index_from_xy(point.x, point. y);
        if(!is_validIndex(index)) continue;
        _costmap.data[index] = _cost_min;
        for(int j = -_lane_width_index_max; j < _lane_width_index_max; j++)
        {
            for(int k = -_lane_width_index_max; k < _lane_width_index_max; k++)
            {
                const int dist2 = j*j+k*k;
                if(dist2 > _lane_width_index_max2) continue;

                const int index_ = index + j*_grid_width + k;
                if(!is_validIndex(index_) || get_y_index_from_index(index) != get_y_index_from_index(index + k)) continue;

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