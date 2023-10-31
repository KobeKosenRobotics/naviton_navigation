#include "nvt_dwa_planner/dwa_planner.h"

DWAPlanner::State::State(double x_, double y_, double yaw_, double velocity_, double yawrate_)
    : x(x_), y(y_), yaw(yaw_), velocity(velocity_), yawrate(yawrate_)
{

}

void DWAPlanner::State::move(const double velocity, const double yawrate, const double dt)
{
    this->yaw += yawrate * dt;
    this->x += velocity * std::cos(this->yaw) * dt;
    this->y += velocity * std::sin(this->yaw) * dt;
    this->velocity = velocity;
    this->yawrate = yawrate;
}

DWAPlanner::Window::Window()
    : min_velocity(0), max_velocity(0), min_yawrate(0), max_yawrate(0)
{

}

DWAPlanner::Window::Window(double min_velocity_, double max_velocity_, double min_yawrate_, double max_yawrate_)
    : min_velocity(min_velocity_), max_velocity(max_velocity_), min_yawrate(min_yawrate_), max_yawrate(max_yawrate_)
{

}

DWAPlanner::DWAPlanner(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_local_goal, topic_local_map, topic_odom, topic_target_velocity, topic_cmd_vel;

    pn.param<std::string>("local_goal_topic", topic_local_goal, "local_goal");
    pn.param<std::string>("local_map_topic", topic_local_map, "local_map");
    pn.param<std::string>("odom_topic", topic_odom, "odom");
    pn.param<std::string>("target_velocity_topic", topic_target_velocity, "target_velocity");
    pn.param<std::string>("cmd_vel_topic", topic_cmd_vel, "cmd_vel");
    
    _local_goal_sub = nh.subscribe(topic_local_goal, 10, &DWAPlanner::local_goal_cb, this);
    _local_map_sub = nh.subscribe(topic_local_map, 10, &DWAPlanner::local_map_cb, this);
    _odom_sub = nh.subscribe(topic_odom, 10, &DWAPlanner::odom_cb, this);
    _target_velocity_sub = nh.subscribe(topic_target_velocity, 10, &DWAPlanner::target_velocity_cb, this);
    _cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic_cmd_vel, 10);

    pn.param<bool>("publish_trajectory", _publish_trajectory, false);
    if(_publish_trajectory)
    {
        std::string topic_trajectory;
        pn.param<std::string>("topic_trajectory", topic_trajectory, "trajectory");
        _trajectory_pub = nh.advertise<visualization_msgs::Marker>(topic_trajectory, 1);
    }

    double frequency;

    pn.param<double>("min_velocity", _min_velocity, 0.0);
    pn.param<double>("max_velocity", _max_velocity, 0.0);
    pn.param<double>("min_yawrate", _min_yawrate, 0.0);
    pn.param<double>("max_yawrate", _max_yawrate, 0.0);
    pn.param<double>("max_acceleration", _max_acceleration, 0.0);
    pn.param<double>("max_d_yawrate", _max_d_yawrate, 0.0);
    pn.param<int>("velocity_resolution", _velocity_resolution, 0.0);
    pn.param<int>("yawrate_resolution", _yawrate_resolution, 0.0);
    pn.param<double>("predict_time", _predict_time, 1.0);
    pn.param<double>("frequency", frequency, 20.0);
    pn.param<double>("error_cost_gain", _error_cost_gain, 0.0);
    pn.param<double>("speed_cost_gain", _speed_cost_gain, 0.0);
    pn.param<double>("costmap_cost_gain", _costmap_cost_gain, 0.0);

    pn.param<double>("obstacle_distance_threshould", _obstacle_distance_threshould, 1.0);
    pn.param<double>("target_velocity", _target_velocity, 0.0);

    _dt = 1.0 / frequency;
    _velocity_resolution_inv = 1.0 / (double)_velocity_resolution;
    _yawrate_resolution_inv = 1.0 / (double)_yawrate_resolution;

    _local_goal_subscribed = _local_map_updated = _odom_updated = false;
}

void DWAPlanner::update()
{
    if(!_local_goal_subscribed) return;
    if(!_local_map_updated) return;
    if(!_odom_updated) return;

    std::vector<DWAPlanner::State> result_trajectory = planning();

    if(_publish_trajectory) visualize_trajectory(result_trajectory);
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = result_trajectory[0].velocity;
    cmd_vel.angular.z = result_trajectory[0].yawrate;

    if(cmd_vel.linear.x < 0 && _local_goal.pose.position.x < 0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = (_local_goal.pose.position.y > 0 ? 1 : -1) * _min_yawrate;
    }
    _cmd_vel_pub.publish(cmd_vel);

    _odom_updated = false;
}

std::vector<DWAPlanner::State> DWAPlanner::planning()
{
    Window window = generate_window();

    double min_cost = 1e6;

    std::vector<DWAPlanner::State> best_trajectory;

    for(int v = 0; v <= _velocity_resolution; v++)
    {
        double velocity = lerp(window.min_velocity, window.max_velocity, v * _velocity_resolution_inv);
        for(int y = 0; y <= _yawrate_resolution; y++)
        {
            double yawrate = lerp(window.min_yawrate, window.max_yawrate, y * _yawrate_resolution_inv);

            // Generate Trajectory
            State state(0.0, 0.0, 0.0, _current_velocity.linear.x, _current_velocity.angular.z);
            std::vector<State> trajectory;
            for(float t = 0.0; t <= _predict_time; t += _dt)
            {
                state.move(velocity, yawrate, _dt);
                trajectory.push_back(state);
            }

            // Calc Cost
            double error_cost = calc_error_cost(trajectory);
            double speed_cost = calc_speed_cost(trajectory);
            double costmap_cost = calc_costmap_cost(trajectory);
            
            if(costmap_cost < 0.0) continue;

            double final_cost = _error_cost_gain * error_cost + _speed_cost_gain * speed_cost + _costmap_cost_gain * costmap_cost;
            if(velocity < 0.0) final_cost *= 2.0;
            if(final_cost > min_cost) continue;
            min_cost = final_cost;

            best_trajectory = trajectory;
        }
    }

    if(min_cost == 1e6)
    {
        best_trajectory.clear();
        State state(0.0, 0.0, 0.0, window.min_velocity, 0.0);
        best_trajectory.push_back(state);
    }

    return best_trajectory;
}

DWAPlanner::Window DWAPlanner::generate_window()
{
    Window window;

    window.min_velocity = std::max(_current_velocity.linear.x - _max_acceleration * _dt, _min_velocity);
    window.max_velocity = std::min(_current_velocity.linear.x + _max_acceleration * _dt, _max_velocity);
    window.min_yawrate = std::max(_current_velocity.angular.z - _max_d_yawrate * _dt, -_max_yawrate);
    window.max_yawrate = std::min(_current_velocity.angular.z + _max_d_yawrate * _dt,  _max_yawrate);

    return window;
}

void DWAPlanner::visualize_trajectory(const std::vector<State>& trajectory)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = _local_map.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0.8;
    marker.ns = _trajectory_pub.getTopic();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.05;
    
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    marker.pose = pose;

    geometry_msgs::Point p;
    
    for(const auto& pose : trajectory)
    {
        p.x = pose.x;
        p.y = pose.y;
        marker.points.push_back(p);
    }

    _trajectory_pub.publish(marker);
}

double DWAPlanner::calc_error_cost(const std::vector<State>& trajectory)
{
    Eigen::Vector3d goal(_local_goal.pose.position.x, _local_goal.pose.position.y, tf::getYaw(_local_goal.pose.orientation));
    Eigen::Vector3d last_position(trajectory.back().x, trajectory.back().y, trajectory.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

double DWAPlanner::calc_speed_cost(const std::vector<State>& trajectory)
{
    float cost = fabs(_target_velocity - fabs(trajectory.back().velocity));
    return cost;
}

double DWAPlanner::calc_costmap_cost(const std::vector<State>& trajectory)
{
    int sign = 1; // 1 : success, -1 : failure

    double cost = 0;
    
    double costmap_threshould = (_obstacle_distance_threshould / (_local_map.info.width * _local_map.info.resolution)) * -198.0 + 100.0;
    if(costmap_threshould < 1) costmap_threshould = 1;

    for(const auto& state : trajectory)
    {
        int index = get_index_from_xy(state.x, state.y);
        if(index == -1 || _local_map.data[index] > costmap_threshould) sign = -1;
        cost += (index != -1) ? _local_map.data[index] : 100;
    }

    return cost * sign;
}

double DWAPlanner::lerp(double a, double b, double t)
{
    t = std::max(std::min(t, 1.0), 0.0);
    return a * (1.0 - t) + b * t; 
}

int DWAPlanner::get_index_from_xy(const double x, const double y)
{
    double resolution = _local_map.info.resolution;
    int grid_width = _local_map.info.width;
    int grid_width_2 = grid_width / 2;

    double width_2 = resolution * grid_width_2;
    if(x < -width_2 || x > width_2 || y < -width_2 || y > width_2) return -1;
    
    const int x_index = floor(x / resolution + 0.5) + grid_width_2;
    const int y_index = floor(y / resolution + 0.5) + grid_width_2;
    return y_index * grid_width + x_index;
}

void DWAPlanner::local_goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    _local_goal = *msg;
    _local_goal_subscribed = true;
}

void DWAPlanner::local_map_cb(const nav_msgs::OccupancyGridConstPtr& msg)
{
    _local_map = *msg;
    _local_map_updated = true;
}

void DWAPlanner::odom_cb(const nav_msgs::OdometryConstPtr& msg)
{
    _current_velocity = msg->twist.twist;
    _odom_updated = true;
}

void DWAPlanner::target_velocity_cb(const geometry_msgs::TwistConstPtr& msg)
{
    _target_velocity = msg->linear.x;
}