#include "trash_detector/trash_detector.h"

TrashDetector::TrashDetector(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
	std::string topic_pc, topic_image, topic_boxes, topic_cam_info, topic_marker;
    pn.param<std::string>("pc_topic", topic_pc, "/d435i/depth/color/points");
	pn.param<std::string>("image_topic", topic_image, "/d435i/aligned_depth_to_color/image_raw");
	pn.param<std::string>("boxes_topic", topic_boxes, "/darknet_ros/bounding_boxes");
	pn.param<std::string>("cam_info_topic", topic_cam_info, "/d435i/aligned_depth_to_color/camera_info");
	pn.param<std::string>("marker_topic", topic_marker, "marker");

    _pc_sub = nh.subscribe(topic_pc, 10, &TrashDetector::pc_cb, this);
	_image_sub = nh.subscribe(topic_image, 10, &TrashDetector::image_cb, this);
	_boxes_sub = nh.subscribe(topic_boxes, 10, &TrashDetector::boxes_cb, this);
	_cam_info_sub = nh.subscribe(topic_cam_info, 10, &TrashDetector::cam_info_cb, this);
	_marker_pub = nh.advertise<visualization_msgs::Marker>(topic_marker, 1);

	_pc_subscribed = false;
	_image_subscribed = false;
	_boxes_subscribed = false;
	_cam_info_subscribed = false;
}

void TrashDetector::update()
{
	if(!_pc_subscribed) return;
	if(!_image_subscribed) return;
	if(!_boxes_subscribed) return;
	if(_boxes_msg.bounding_boxes.size() < 1) return;
	if(!_cam_info_subscribed) return;

	int bottle_index = -1;
	double probability_max = 0.0;
	for(int i = 0; i < _boxes_msg.bounding_boxes.size(); i++)
	{
		if(_boxes_msg.bounding_boxes[i].Class != "bottle") continue;
		if(_boxes_msg.bounding_boxes[i].probability < probability_max) continue;
		bottle_index = i;
		probability_max = _boxes_msg.bounding_boxes[i].probability;
	}
	if(bottle_index == -1) return;

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(_image_msg, _image_msg.encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat &mat = cv_ptr->image;

	int px = (_boxes_msg.bounding_boxes[bottle_index].xmin + _boxes_msg.bounding_boxes[bottle_index].xmax) / 2;
	int py =(_boxes_msg.bounding_boxes[bottle_index].ymin + _boxes_msg.bounding_boxes[bottle_index].ymax) / 2;

	float depth = (double)(mat.at<uint16_t>(py, px)) * 0.001;
	float x = (px - _cam_info_msg.K[2]) / _cam_info_msg.K[0];
	float y = (py - _cam_info_msg.K[5]) / _cam_info_msg.K[4];
	float r2  = x*x + y*y;
	float f = 1 + _cam_info_msg.D[0]*r2 + _cam_info_msg.D[1]*r2*r2 + _cam_info_msg.D[4]*r2*r2*r2;
	float ux = x*f + 2*_cam_info_msg.D[2]*x*y + _cam_info_msg.D[3]*(r2 + 2*x*x);
	float uy = y*f + 2*_cam_info_msg.D[3]*x*y + _cam_info_msg.D[2]*(r2 + 2*y*y);

	std::cout << std::to_string(depth * ux) << ", " << std::to_string(depth * uy) << ", " << std::to_string(depth) << std::endl;

	visualization_msgs::Marker marker;
	marker.header.frame_id = _image_msg.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0; 
	marker.pose.position.x = depth * ux;
	marker.pose.position.y = depth * uy;
	marker.pose.position.z = depth;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	_marker_pub.publish(marker);

	_pc_subscribed = false;
	_boxes_subscribed = false;
}

void TrashDetector::pc_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    _pc_msg = *msg;
	_pc_subscribed = true;
}

void TrashDetector::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
	_image_msg = *msg;
	_image_subscribed = true;
}

void TrashDetector::boxes_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
	_boxes_msg = *msg;
	_boxes_subscribed = true;
}

void TrashDetector::cam_info_cb(const sensor_msgs::CameraInfoConstPtr& msg)
{
	_cam_info_msg = *msg;
	_cam_info_subscribed = true;
}