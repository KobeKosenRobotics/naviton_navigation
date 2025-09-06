#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_srvs/SetBool.h>

class SafetyPlanner
{
public:
    SafetyPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    {
        pnh.param<std::string>("cmd_vel_raw", cmd_vel_raw_topic_, "/cmd_vel_raw");
        pnh.param<std::string>("scan_cloud", scan_cloud_topic_, "/scan_cloud");
        pnh.param<std::string>("cmd_vel", cmd_vel_topic_, "/cmd_vel");

        pnh.param("front_min_x", front_min_x_, 0.0);
        pnh.param("front_max_x", front_max_x_, 1.0);
        pnh.param("front_min_y", front_min_y_, -0.5);
        pnh.param("front_max_y", front_max_y_, 0.5);
        pnh.param("front_min_z", front_min_z_, -0.5);
        pnh.param("front_max_z", front_max_z_, 1.5);

        pub_cmd_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        sub_cmd_raw_ = nh_.subscribe(cmd_vel_raw_topic_, 1, &SafetyPlanner::cmdRawCb, this);
        sub_cloud_ = nh_.subscribe(scan_cloud_topic_, 1, &SafetyPlanner::cloudCb, this);

        obstacle_present_ = false;
    paused_ = true;

    // service to set pause
    srv_pause_ = nh_.advertiseService("/safety_planner/set_pause", &SafetyPlanner::setPauseCb, this);
    }

    void cmdRawCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        latest_cmd_ = *msg;
    }

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        bool obstacle = false;
        for(const auto &pt : cloud.points)
        {
            if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
            if(pt.x >= front_min_x_ && pt.x <= front_max_x_ &&
               pt.y >= front_min_y_ && pt.y <= front_max_y_ &&
               pt.z >= front_min_z_ && pt.z <= front_max_z_)
            {
                obstacle = true;
                break;
            }
        }
        obstacle_present_ = obstacle;
    }

    void spin()
    {
        ros::Rate rate(10);
        while(ros::ok())
        {
            geometry_msgs::Twist out;
            if(paused_ || obstacle_present_)
            {
                // zero velocities
            }
            else
            {
                out = latest_cmd_;
            }
            pub_cmd_.publish(out);
            ros::spinOnce();
            rate.sleep();
        }
    }

    bool setPauseCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        paused_ = req.data;
        res.success = true;
        res.message = paused_ ? "safety_planner paused" : "safety_planner resumed";
        return true;
    }

private:
    ros::NodeHandle nh_;
    std::string cmd_vel_raw_topic_;
    std::string scan_cloud_topic_;
    std::string cmd_vel_topic_;

    double front_min_x_, front_max_x_;
    double front_min_y_, front_max_y_;
    double front_min_z_, front_max_z_;

    ros::Publisher pub_cmd_;
    ros::Subscriber sub_cmd_raw_;
    ros::Subscriber sub_cloud_;

    geometry_msgs::Twist latest_cmd_;
    bool obstacle_present_;
    bool paused_;
    ros::ServiceServer srv_pause_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    SafetyPlanner node(nh, pnh);
    node.spin();
    return 0;
}
