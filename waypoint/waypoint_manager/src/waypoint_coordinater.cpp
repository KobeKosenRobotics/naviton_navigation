void WaypointCoordinater::publish()
{
    // 現在の位置を取得（base_link -> map のTFを取得）
    geometry_msgs::Pose current_wp_pose;
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform("map", "naviton/base_link", ros::Time(0), ros::Duration(1.0));
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

    // 任意のウェイポイントを取得
    int wp_number = 1;  // 例えば、3番目のウェイポイント
    if (wp_number >= _wps.waypoints.size())
    {
        ROS_WARN("Waypoint index out of range");
        return;
    }
    waypoint_msgs::waypoint waypoint_to_calculate = _wps.waypoints[wp_number];

    // 相対座標を計算
    geometry_msgs::Pose relative_pose = calculateRelativePosition(waypoint_to_calculate.pose.pose, current_wp_pose);

    // 絶対座標を計算（相対座標をmap座標系に変換）
    geometry_msgs::PoseStamped absolute_pose;
    absolute_pose.header.stamp = ros::Time::now();
    absolute_pose.header.frame_id = "map";  // map座標系でパブリッシュ

    // tf2を使って座標変換
    tf2::Transform tf_absolute;
    tf_absolute.setOrigin(_absolute_position.getOrigin());  // YAMLで設定した絶対座標
    tf_absolute.setRotation(_absolute_position.getRotation());

    tf2::Vector3 relative_position(relative_pose.position.x/100000, relative_pose.position.y/100000, relative_pose.position.z);
    tf2::Vector3 absolute_position = tf_absolute * relative_position;

    absolute_pose.pose.position.x = absolute_position.x();
    absolute_pose.pose.position.y = absolute_position.y();
    absolute_pose.pose.position.z = absolute_position.z();

    // 絶対オリエンテーションの計算
    tf2::Quaternion relative_quat, abs_quat;
    tf2::fromMsg(relative_pose.orientation, relative_quat);
    abs_quat = _absolute_position.getRotation();

    tf2::Quaternion final_orientation = abs_quat * relative_quat;
    absolute_pose.pose.orientation = tf2::toMsg(final_orientation);

    // 絶対座標をパブリッシュ
    abs_wp_publisher.publish(absolute_pose);
}
