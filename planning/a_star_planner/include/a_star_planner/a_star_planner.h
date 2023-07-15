#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>

#include "a_star_solver.h"

class AStarPlanner
{
    public:
        AStarPlanner(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void publish();
    private:
        void costmap_cb(nav_msgs::OccupancyGridConstPtr msg);
        void goal_cb(geometry_msgs::PoseStampedConstPtr msg);

        AStarSolver _solver;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener* tf2_listener;

        ros::Subscriber _costmap_subscriber;
        ros::Subscriber _goal_subscriber;
        ros::Publisher _path_publisher;

        std::string _robot_frame_id = "base_link";
        nav_msgs::MapMetaData _mapData;

        bool _costmapUpdated = false;
        bool _goalUpdated = false;

        AStarSolver::Vector2Int _source;
};

#endif