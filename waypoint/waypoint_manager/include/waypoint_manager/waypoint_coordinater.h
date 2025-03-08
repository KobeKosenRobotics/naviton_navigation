#ifndef WAYPOINT_COORDINATER_H
#define WAYPOINT_COORDINATER_H

#include <geometry_msgs/PoseStamped.h>
#include <waypoint_msgs/waypoint.h>
#include <waypoint_msgs/waypoints.h>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>  // tfライブラリのインクルード
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>  // TFリスナー用インクルード
#include <tf2_ros/buffer.h>  // TFバッファ用インクルード

class WaypointCoordinater
{
public:
    // コンストラクタ
    WaypointCoordinater(ros::NodeHandle &nh, ros::NodeHandle &pn);

    // 相対位置計算
    geometry_msgs::Pose calculateRelativePosition(const geometry_msgs::Pose &target, const geometry_msgs::Pose &current);

    // 絶対座標をパブリッシュ
    void publish();

private:
    ros::Publisher abs_wp_publisher;  // 絶対座標をパブリッシュするためのパブリッシャ
    tf2::Transform _absolute_position; // 絶対座標用の変数
    int _index_now;                  // 現在のウェイポイントインデックス
    waypoint_msgs::waypoints _wps;  // ウェイポイントリスト
    waypoint_msgs::waypoint waypoint_to_calculate;

    ros::Subscriber waypoints_sub;
    void waypointsCallback(const waypoint_msgs::waypoints::ConstPtr &msg);

    // TFリスナー関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;  // TFバッファ
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;  // TFリスナー
};

#endif // WAYPOINT_COORDINATER_H
