#include "lrf_door_detector/door_detector.h"

DoorDetector::DoorDetector(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_pc, topic_pc_filtered;
    pn.param<std::string>("pc_topic", topic_pc, "pc_topic");
    pn.param<std::string>("pc_filtered_topic", topic_pc_filtered, "pc_filtered_topic");
    _pc_sub = nh.subscribe(topic_pc, 10, &DoorDetector::pc_cb, this);
    _pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_pc_filtered, 10);

    _pc_subscribed = false;
}

void DoorDetector::update()
{
    if(!_pc_subscribed) return;
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(_pc_input.makeShared());
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ> pc_extracted;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(_pc_input.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(pc_extracted);
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc_extracted, msg);
    _pc_pub.publish(msg);

    _pc_subscribed = false;
}

void DoorDetector::pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, _pc_input);
    _pc_subscribed = true;
}