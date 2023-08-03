#include "a_star_planner/a_star_planner.h"

AStarPlanner::AStarPlanner(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    tf2_listener = new tf2_ros::TransformListener(tfBuffer);

    std::string topic_costmap_sub, topic_goal_sub, topic_path_pub;
    
    pn.param<std::string>("costmap_topic", topic_costmap_sub, "local_costmap");
    pn.param<std::string>("goal_topic", topic_goal_sub, "local_goal");
    pn.param<std::string>("path_topic", topic_path_pub, "local_path");

    _costmap_subscriber = nh.subscribe(topic_costmap_sub, 10, &AStarPlanner::costmap_cb, this);
    _goal_subscriber = nh.subscribe(topic_goal_sub, 10, &AStarPlanner::goal_cb, this);

    _path_publisher = nh.advertise<nav_msgs::Path>(topic_path_pub, 1);

    pn.param<std::string>("robot_frame_id", _robot_frame_id, "base_link");

    _costmapUpdated = false;
    _goalUpdated = false;
}

void AStarPlanner::publish()
{
    if(!_costmapUpdated || !_goalUpdated) return;
    if(_solver.Solve(5))
    {
        nav_msgs::Path path;
        path.header.frame_id = _robot_frame_id;
        for(AStarSolver::Vector2Int &p : _solver.path)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _robot_frame_id;
            pose.pose.position.x = (p.x - _source.x) * _mapData.resolution;
            pose.pose.position.y = (p.y - _source.y) * _mapData.resolution;
            path.poses.push_back(pose);
        }
        _path_publisher.publish(path);
    }
    else
    {
        std::cout << "Not solved" << std::endl;
    }
    _costmapUpdated = _goalUpdated = false;
}

void AStarPlanner::costmap_cb(nav_msgs::OccupancyGridConstPtr msg)
{
    if(msg->header.frame_id != _robot_frame_id)
    {
        ROS_WARN("Costmap frame_id is invalid.");
        return;
    }

    _mapData = msg->info;

    _source.x = (int)(_mapData.width / 2 + 0.5f);
    _source.y = (int)(_mapData.height / 2 + 0.5f);
    _solver.SetSource(_source);
    _solver.SetMap(msg);
    _costmapUpdated = true;
}

void AStarPlanner::goal_cb(geometry_msgs::PoseStampedConstPtr msg)
{   
    geometry_msgs::Pose pose = msg->pose;

    if(msg->header.frame_id != _robot_frame_id)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tfBuffer.lookupTransform(_robot_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        tf2::doTransform(pose, pose, transform_stamped);
    }

    AStarSolver::Vector2Int target
    {
        (int)(pose.position.x / _mapData.resolution + 0.5f),
        (int)(pose.position.y / _mapData.resolution + 0.5f)
    };
    _solver.SetTarget(target + _source);
    _goalUpdated = true;
}