#include "waypoint_manager/waypoint_loader.h"

WaypointLoader::WaypointLoader(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    std::string topic_wps;
    pn.param<std::string>("topic_waypoints", topic_wps, "wpLoader/waypoints");
    _wps_publisher = nh.advertise<waypoint_msgs::waypoints>(topic_wps, 1);

    pn.param<std::string>("frame_id_map", _frame_id_map, "map");

    _wps.header.frame_id = _frame_id_map;
}

void WaypointLoader::load(std::string file_path)
{
    std::cout << "WaypointLoader: Load" << std::endl;
    std::cout << "\tFilePath: " << file_path << std::endl;

    _wps.waypoints.clear();

    std::string buffer;
    std::string comma_buffer;
    
    std::ifstream ifs_csv_file(file_path);
    getline(ifs_csv_file, buffer);

    int index = 0;
    while (getline(ifs_csv_file, buffer))
    {   
        std::istringstream i_stream(buffer);
        
        waypoint_msgs::waypoint wp;
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.index = index;

        wp.pose.header.frame_id = _frame_id_map;
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.position.x = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.position.y = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.position.z = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.orientation.x = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.orientation.y = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.orientation.z = std::stod(comma_buffer);
        if(!getline(i_stream, comma_buffer, ',')) continue;
        wp.pose.pose.orientation.w = std::stod(comma_buffer);

        while (getline(i_stream, comma_buffer, ','))
        {
            std::istringstream i_stream_(comma_buffer);
            std::string type;
            if(!getline(i_stream_, type, ':')) continue;
            std::string value;
            if(!getline(i_stream_, value, ':')) continue;
            waypoint_msgs::waypoint_attribute attribute;
            attribute.type = std::stod(type);
            attribute.value = std::stod(value);
            wp.attributes.push_back(attribute);
        }

        _wps.waypoints.push_back(wp);
        index++;
    }
}

void WaypointLoader::publish()
{
    _wps_publisher.publish(_wps);
}