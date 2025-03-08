#include "waypoint_manager/waypoint_coordinater.h"

WaypointCoordinater::WaypointCoordinater(ros::NodeHandle &nh, ros::NodeHandle &pn)
{
    // トピック名やサービス名をパラメータから取得
    std::string topic_absolute_position;
    pn.param<std::string>("topic_absolute_position", topic_absolute_position, "wpManager/absolute_position");
    abs_wp_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_absolute_position, 10);

    std::string topic_waypoints;
    pn.param<std::string>("topic_waypoints", topic_waypoints, "wpLoader/waypoints");
    waypoints_sub = nh.subscribe(topic_waypoints, 10, &WaypointCoordinater::waypointsCallback, this);
    
    // YAMLファイルから任意の絶対座標とオリエンテーションを設定
    double absolute_x, absolute_y, absolute_z;
    double absolute_qx, absolute_qy, absolute_qz, absolute_qw;
    
    pn.param("absolute_x", absolute_x, 0.0);
    pn.param("absolute_y", absolute_y, 0.0);
    pn.param("absolute_z", absolute_z, 0.0);
    pn.param("absolute_orientation_x", absolute_qx, 0.0);
    pn.param("absolute_orientation_y", absolute_qy, 0.0);
    pn.param("absolute_orientation_z", absolute_qz, 0.0);
    pn.param("absolute_orientation_w", absolute_qw, 1.0);

    // _absolute_positionをtf2::Transformで宣言し、座標と回転を設定
    tf2::Quaternion abs_quat(absolute_qx, absolute_qy, absolute_qz, absolute_qw);
    _absolute_position.setOrigin(tf2::Vector3(absolute_x, absolute_y, absolute_z));
    _absolute_position.setRotation(abs_quat);

    // TFバッファとリスナーの初期化
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

geometry_msgs::Pose WaypointCoordinater::calculateRelativePosition(const geometry_msgs::Pose &target, const geometry_msgs::Pose &current)
{
    geometry_msgs::Pose relative_pose;

    // 目標位置と現在位置の相対座標計算
    relative_pose.position.x = target.position.x - current.position.x;
    relative_pose.position.y = target.position.y - current.position.y;
    relative_pose.position.z = target.position.z - current.position.z;

    // 回転の計算（クォータニオンの相対回転）
    tf2::Quaternion q_target, q_current, q_relative;
    tf2::fromMsg(target.orientation, q_target);  // geometry_msgs::Quaternion -> tf2::Quaternion
    tf2::fromMsg(current.orientation, q_current);

    // 相対回転 = 現在の回転の逆 × 目標の回転
    q_relative = q_current.inverse() * q_target;

    // 計算したクォータニオンを代入
    relative_pose.orientation = tf2::toMsg(q_relative);  // tf2::Quaternion -> geometry_msgs::Quaternion

    return relative_pose;
}

void WaypointCoordinater::waypointsCallback(const waypoint_msgs::waypoints::ConstPtr &msg)
{
    _wps = *msg; // 受け取ったデータを保存
    ROS_INFO("Waypoints received: %zu waypoints", _wps.waypoints.size());
}

void WaypointCoordinater::publish()
{
    // 現在の位置（tf2を使ってbase_link座標を取得）
    geometry_msgs::Pose current_wp_pose;
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform("naviton/base_link", "map", ros::Time(0), ros::Duration(1.0));
        current_wp_pose.position.x = transform.transform.translation.x;
        current_wp_pose.position.y = transform.transform.translation.y;
        current_wp_pose.position.z = transform.transform.translation.z;
        current_wp_pose.orientation = transform.transform.rotation;
    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN("Transform failed: %s", e.what());
        return;
    }

    // 任意のウェイポイント番号を指定し、その座標を基に相対座標を計算
    int wp_number = 1;  // 任意のウェイポイント番号（例えば、3番目のウェイポイント）
    waypoint_msgs::waypoint waypoint_to_calculate = _wps.waypoints[wp_number];

    // 相対位置を計算
    geometry_msgs::Pose relative_pose = calculateRelativePosition(waypoint_to_calculate.pose.pose, current_wp_pose);

    // 絶対座標を計算（YAML設定された絶対座標を基に相対座標を足す）
    geometry_msgs::PoseStamped absolute_pose;

    // tf2ベクトルに変換して回転を適用
    tf2::Vector3 relative_position(relative_pose.position.x*10000, relative_pose.position.y*10000, relative_pose.position.z*10000);
    tf2::Vector3 absolute_position = _absolute_position.getOrigin() + relative_position;

    absolute_pose.pose.position.x = absolute_position.x();
    absolute_pose.pose.position.y = absolute_position.y();
    absolute_pose.pose.position.z = absolute_position.z();

    // 絶対オリエンテーションの計算
    tf2::Quaternion abs_quat = _absolute_position.getRotation();

    // 相対回転を取得し、最終的なオリエンテーションを計算
    tf2::Quaternion relative_quat;
    tf2::fromMsg(relative_pose.orientation, relative_quat);
    tf2::Quaternion final_orientation = abs_quat * relative_quat;

    absolute_pose.pose.orientation = tf2::toMsg(final_orientation);

    // 絶対座標をパブリッシュ
    abs_wp_publisher.publish(absolute_pose);
}
