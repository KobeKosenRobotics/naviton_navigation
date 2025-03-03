#include "waypoint_manager/waypoint_coordinater.h"

WaypointCoordinater::WaypointCoordinater(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    // トピック名やサービス名をパラメータから取得
    std::string topic_absolute_position;
    pn.param<std::string>("topic_absolute_position", topic_absolute_position, "wpManager/absolute_position");
    now_wp_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_absolute_position, 10);

    // YAMLファイルから任意の絶対座標を設定
    double absolute_x, absolute_y, absolute_z;
    pn.param("absolute_x", absolute_x, 0.0); // デフォルトは (0, 0, 0)
    pn.param("absolute_y", absolute_y, 0.0);
    pn.param("absolute_z", absolute_z, 0.0);

    // _absolute_positionをtf::Transformで宣言し、座標を設定
    _absolute_position.setOrigin(tf::Vector3(absolute_x, absolute_y, absolute_z)); // tf座標系で使用する場合に対応
}

geometry_msgs::Pose WaypointCoordinater::calculateRelativePosition(const geometry_msgs::Pose &target, const geometry_msgs::Pose &current)
{
    geometry_msgs::Pose relative_pose;
    
    // 目標位置と現在位置の相対座標計算
    relative_pose.position.x = target.position.x - current.position.x;
    relative_pose.position.y = target.position.y - current.position.y;
    relative_pose.position.z = target.position.z - current.position.z;
    
    // 回転（姿勢）も考慮する場合はクォータニオンで計算することも可能
    // relative_pose.orientation = some_rotation_calculation;
    
    return relative_pose;
}

void WaypointCoordinater::publish()
{
    if (_index_now < 0 || _wps.waypoints.size() <= _index_now)
        return;

    // 現在の目標ウェイポイント
    waypoint_msgs::waypoint target_wp = _wps.waypoints[_index_now];
    
    // 現在の位置（仮に現在位置を取得したものとします）
    waypoint_msgs::waypoint current_wp; // 現在位置は取得する必要あり

    // 任意のウェイポイント番号を指定し、その座標を基に相対座標を計算
    int wp_number = 2;  // 任意のウェイポイント番号（例えば、3番目のウェイポイント）
    waypoint_msgs::waypoint waypoint_to_calculate = _wps.waypoints[wp_number];

    // 相対位置を計算
    geometry_msgs::Pose relative_pose = calculateRelativePosition(waypoint_to_calculate.pose.pose, current_wp.pose.pose);
    
    // 絶対座標を計算（YAML設定された絶対座標を基に相対座標を足す）
    geometry_msgs::PoseStamped absolute_pose;
    absolute_pose.pose.position.x = _absolute_position.getOrigin().getX() + relative_pose.position.x;
    absolute_pose.pose.position.y = _absolute_position.getOrigin().getY() + relative_pose.position.y;
    absolute_pose.pose.position.z = _absolute_position.getOrigin().getZ() + relative_pose.position.z;

    // 絶対座標をパブリッシュ
    now_wp_publisher.publish(absolute_pose);
}
