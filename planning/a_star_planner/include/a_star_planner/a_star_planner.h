#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>

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

        nav_msgs::OccupancyGrid _costmap;
        geometry_msgs::PoseStamped _goal;

        ros::Publisher _cmd_vel_publisher;

        bool _updated = false;

        const float _dir_x[8] = {0};
        const float _dir_y[8] = {0};
};

#endif