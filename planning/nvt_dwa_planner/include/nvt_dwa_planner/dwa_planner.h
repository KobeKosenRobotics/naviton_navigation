#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

class DWAPlanner
{
    public:
        class State
        {
            public:
                State(double x_, double y_, double yaw_, double velocity_, double yawrate_);
                void move(const double velocity, const double yawrate, const double dt);

                double x;
                double y;
                double yaw;
                double velocity;
                double yawrate;
        };

        class Window
        {
            public:
                Window();
                Window(double min_velocity_, double max_velocity_, double min_yawrate_, double max_yawrate_);
                double min_velocity;
                double max_velocity;
                double min_yawrate;
                double max_yawrate;
        };

        DWAPlanner(ros::NodeHandle &nh, ros::NodeHandle &pn);
        
        void update();
        std::vector<State> planning();
        
        Window generate_window();
        void visualize_trajectory(const std::vector<State>& trajectory);
        
        double calc_error_cost(const std::vector<State>& trajectory);
        double calc_speed_cost(const std::vector<State>& trajectory);
        double calc_costmap_cost(const std::vector<State>& trajectory);

        double lerp(double a, double b, double t);
        int get_index_from_xy(const double x, const double y);
        
        void local_goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void local_map_cb(const nav_msgs::OccupancyGridConstPtr& msg);
        void odom_cb(const nav_msgs::OdometryConstPtr& msg);
        void target_velocity_cb(const geometry_msgs::TwistConstPtr& msg);

    private:
        ros::Subscriber _local_goal_sub, _local_map_sub, _odom_sub, _target_velocity_sub;
        ros::Publisher _cmd_vel_pub;

        ros::Publisher _trajectory_pub;

        double _min_linear_speed;
        double _min_angular_speed;
        double _min_velocity;
        double _max_velocity;

        double _min_yawrate;
        double _max_yawrate;
        double _max_acceleration;
        double _max_d_yawrate;

        int _velocity_resolution;
        int _yawrate_resolution;

        double _predict_time;
        double _dt;

        double _error_cost_gain;
        double _speed_cost_gain;
        double _costmap_cost_gain;

        geometry_msgs::PoseStamped _local_goal;
        nav_msgs::OccupancyGrid _local_map;
        geometry_msgs::Twist _current_velocity;
        double _target_velocity;
        double _obstacle_distance_threshould;
        bool _local_goal_subscribed;
        bool _local_map_updated;
        bool _odom_updated;

        bool _publish_trajectory;

        double _velocity_resolution_inv;
        double _yawrate_resolution_inv;
};

#endif
