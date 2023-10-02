#include "a_star_planner/a_star_planner.h"

AStarPlanner::AStarPlanner(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    tf2_listener = new tf2_ros::TransformListener(tfBuffer);

    std::string topic_costmap_sub, topic_goal_sub, topic_path_pub, topic_goal_pub;
    
    pn.param<std::string>("costmap_topic", topic_costmap_sub, "local_costmap");
    pn.param<std::string>("goal_topic", topic_goal_sub, "local_goal");
    pn.param<std::string>("path_topic", topic_path_pub, "local_path");
    pn.param<std::string>("a_star_goal_topic", topic_goal_pub, "a_star_goal");
    _costmap_subscriber = nh.subscribe(topic_costmap_sub, 10, &AStarPlanner::costmap_cb, this);
    _goal_subscriber = nh.subscribe(topic_goal_sub, 10, &AStarPlanner::goal_cb, this);

    _path_publisher = nh.advertise<nav_msgs::Path>(topic_path_pub, 1);
    _goal_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_goal_pub, 1);

    pn.param<std::string>("robot_frame_id", _robot_frame_id, "base_link");
    pn.param<double>("goal_distance", _goal_distance, 1.0);

    _costmapUpdated = false;
    _goalUpdated = false;
}

void AStarPlanner::publish()
{
    if(!_costmapUpdated || !_goalUpdated) return;
    if(_solver.Solve(40))
    {
        nav_msgs::Path path;
        path.header.frame_id = _robot_frame_id;

        double pos_x_last = 0.0;
        double pos_y_last = 0.0;
        
        geometry_msgs::PoseStamped goal;
        double distance = 0.0;
        for(AStarSolver::Vector2Int &p : _solver.path)
        {
            double pos_x = (p.x - _source.x) * _mapData.resolution;
            double pos_y = (p.y - _source.y) * _mapData.resolution;
            double dx = pos_x - pos_x_last;
            double dy = pos_y - pos_y_last;
            
            geometry_msgs::PoseStamped pose;
            
            pose.header.frame_id = _robot_frame_id;
            pose.pose.position.x = pos_x;
            pose.pose.position.y = pos_y;

            double angle = atan2(dy, dx);
            if(angle < -M_PI) angle += 2*M_PI;
            else if(angle > M_PI) angle -= 2*M_PI;
            pose.pose.orientation = euler2quat(0, 0, angle);

            path.poses.push_back(pose);

            if(distance < _goal_distance)
            {
                goal = pose;
                distance += sqrt(dx*dx + dy*dy);
            }

            pos_x_last = pos_x;
            pos_y_last = pos_y;
        }
        _goal_publisher.publish(goal);
        _path_publisher.publish(path);
    }
    else
    {
        std::cout << "Not solved" << std::endl;
    }
    _costmapUpdated = _goalUpdated = false;
}

geometry_msgs::Quaternion AStarPlanner::euler2quat(double roll, double pitch, double yaw){
    //q0:w q1:x q2:y q3:z
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);
    geometry_msgs::Quaternion quat;
    quat.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    quat.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    quat.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    quat.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    return quat;
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