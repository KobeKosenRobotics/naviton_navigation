#include "waypoint_manager/waypoint_recorder.h"

WaypointRecorder::WaypointRecorder(ros::NodeHandle &nh, ros::NodeHandle &pn) : _buffer(), _listener(_buffer)
{
    std::string topic_wps;
    pn.param<std::string>("topic_waypoints", topic_wps, "wpRecorder/waypoints");
    _wps_publisher = nh.advertise<waypoint_msgs::waypoints>(topic_wps, 1);

    std::string service_recorder_start;
    pn.param<std::string>("service_recorder_start", service_recorder_start, "wpRecorder/start");
    _recorder_start_server = nh.advertiseService(service_recorder_start, &WaypointRecorder::recorder_start_cb, this);

    std::string service_recorder_end;
    pn.param<std::string>("service_recorder_end", service_recorder_end, "wpRecorder/end");
    _recorder_end_server = nh.advertiseService(service_recorder_end, &WaypointRecorder::recorder_end_cb, this);

    std::string service_recorder_record;
    pn.param<std::string>("service_recorder_record", service_recorder_record, "wpRecorder/record");
    _recorder_record_server = nh.advertiseService(service_recorder_record, &WaypointRecorder::recorder_record_cb, this);

    std::string service_recorder_attributes;
    pn.param<std::string>("service_recorder_attributes", service_recorder_attributes, "wpRecorder/attributes");
    _recorder_attributes_server = nh.advertiseService(service_recorder_attributes, &WaypointRecorder::recorder_attributes_cb, this);

    int updateFrequency;
    pn.param<int>("frequency", updateFrequency, 10);
    double updateTimerDuration = 1.0f/((double)updateFrequency);

    pn.param<double>("lookupTimeout", _lookupTimeout, 0.2);

    double pitch;
    pn.param<double>("pitch", pitch, 1.0);
    _pitch2 = pitch*pitch;

    pn.param<std::string>("frame_id_robot", _frame_id_robot, "base_link");
    pn.param<std::string>("frame_id_map", _frame_id_map, "map");

    pn.param<std::string>("file_dir", _file_dir, "/home");

    _timer = nh.createTimer(ros::Duration(updateTimerDuration), [&](const ros::TimerEvent& e) 
    {
        if(!_recording) return;
        try
        {
            _tf_stamped                     =   _buffer.lookupTransform(_frame_id_map, _frame_id_robot, ros::Time(0), ros::Duration(_lookupTimeout));
            _pose_stamped.header            =   _tf_stamped.header;
            _pose_stamped.header.frame_id   =   _frame_id_map;
            _pose_stamped.pose.position.x   =   _tf_stamped.transform.translation.x;
            _pose_stamped.pose.position.y   =   _tf_stamped.transform.translation.y;
            _pose_stamped.pose.position.z   =   _tf_stamped.transform.translation.z;
            _pose_stamped.pose.orientation  =   _tf_stamped.transform.rotation;
            record();
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    });

    _wps.header.frame_id = _frame_id_map;
    _wps.header.seq = 0;
    _wpIndex = 0;
}

bool WaypointRecorder::recorder_start_cb(waypoint_manager_msgs::waypoint_recorder_start::Request& req, waypoint_manager_msgs::waypoint_recorder_start::Response& res)
{
    if(_recording) return false;
    std::cout << "WaypointRecorder: Start" << std::endl;
    
    _wps.waypoints.clear();
    _attributes_permanent.clear();
    _recording = true;

    _point_old.x = _point_old.y = _point_old.z = DBL_MAX;
    return true;
}


bool WaypointRecorder::recorder_end_cb(waypoint_manager_msgs::waypoint_recorder_end::Request& req, waypoint_manager_msgs::waypoint_recorder_end::Response& res)
{
    if(!_recording) return false;
    _recording = false;
    std::cout << "WaypointRecorder: End" << std::endl;
    save("wpData_latest");
    if(!req.save) return true;
    std::cout << "WaypointRecorder: Save" << std::endl;
    save(req.file_name);
    return true;
}

void WaypointRecorder::save(std::string file_name)
{
    std::ofstream ofs_csv_file(_file_dir +  "/" + file_name + ".csv", std::ios::trunc);

    ofs_csv_file << "number" << ',';
    ofs_csv_file << "x" << ',';
    ofs_csv_file << "y" << ',';
    ofs_csv_file << "z" << ',';
    ofs_csv_file << "qx" << ',';
    ofs_csv_file << "qy" << ',';
    ofs_csv_file << "qz" << ',';
    ofs_csv_file << "qw" << ',';
    ofs_csv_file << "attributes" << std::endl;

    for(int i = 0; i < _wps.waypoints.size(); i++)
    {
        const auto& wp = _wps.waypoints[i];
        const auto& pose = wp.pose.pose;
        ofs_csv_file << i << ',';
        ofs_csv_file << pose.position.x << ',';
        ofs_csv_file << pose.position.y << ',';
        ofs_csv_file << pose.position.z << ',';
        ofs_csv_file << pose.orientation.x << ',';
        ofs_csv_file << pose.orientation.y << ',';
        ofs_csv_file << pose.orientation.z << ',';
        ofs_csv_file << pose.orientation.w << ',';
        for (int j = 0; j < _wps.waypoints[i].attributes.size(); j++)
        {
            ofs_csv_file << _wps.waypoints[i].attributes[j].type << ':' << _wps.waypoints[i].attributes[j].value << ',';
        }
        ofs_csv_file << "" << std::endl;
    }
}

bool WaypointRecorder::recorder_record_cb(waypoint_manager_msgs::waypoint_recorder_record::Request& req, waypoint_manager_msgs::waypoint_recorder_record::Response& res)
{
    if(!_recording) return false;

    std::cout << "WaypointRecorder: AddRecord" << std::endl;

    try
    {
        _tf_stamped                     =   _buffer.lookupTransform(_frame_id_robot, _frame_id_map, ros::Time(0), ros::Duration(_lookupTimeout));
        _pose_stamped.header            =   _tf_stamped.header;
        _pose_stamped.header.frame_id   =   _frame_id_map;
        _pose_stamped.pose.position.x   =   _tf_stamped.transform.translation.x;
        _pose_stamped.pose.position.y   =   _tf_stamped.transform.translation.y;
        _pose_stamped.pose.position.z   =   _tf_stamped.transform.translation.z;
        _pose_stamped.pose.orientation  =   _tf_stamped.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }

    record(req.attributes);
    return true;
}

bool WaypointRecorder::recorder_attributes_cb(waypoint_manager_msgs::waypoint_recorder_attributes::Request& req, waypoint_manager_msgs::waypoint_recorder_attributes::Response& res)
{
    if(!_recording) return false;
    
    std::cout << "WaypointRecorder: SetAttributes" << std::endl;

    _attributes_permanent.clear();

    for(int i = 0; i < req.attributes.size(); i++)
    {
        _attributes_permanent.push_back(req.attributes[i]);
    }
    return true;
}

void WaypointRecorder::record()
{
    const auto& point = _pose_stamped.pose.position;

    double diff_x = point.x - _point_old.x;
    double diff_y = point.y - _point_old.y;
    double diff_z = point.z - _point_old.z;
    if(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z < _pitch2) return;

    _point_old.x = point.x;
    _point_old.y = point.y;
    _point_old.z = point.z;

    waypoint_msgs::waypoint wp;
    wp.index = _wpIndex;
    wp.pose = _pose_stamped;
    for(const waypoint_msgs::waypoint_attribute &attribute : _attributes_permanent)
    {
        wp.attributes.push_back(attribute);
    }

    _wps.waypoints.push_back(wp);

    _wpIndex++;


    publishPath();
}

void WaypointRecorder::record(std::vector<waypoint_msgs::waypoint_attribute> attributes)
{
    const auto& point = _pose_stamped.pose.position;

    _point_old.x = point.x;
    _point_old.y = point.y;
    _point_old.z = point.z;

    waypoint_msgs::waypoint wp;
    wp.index = _wpIndex;
    wp.pose = _pose_stamped;
    for(const waypoint_msgs::waypoint_attribute &attribute : attributes)
    {
        wp.attributes.push_back(attribute);
    }

    _wps.waypoints.push_back(wp);

    _wpIndex++;

    publishPath();
}

void WaypointRecorder::publishPath()
{
    _wps.header.stamp = ros::Time::now();
    _wps_publisher.publish(_wps);
    _wps.header.seq++;
}