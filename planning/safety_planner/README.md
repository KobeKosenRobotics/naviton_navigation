safety_planner
===============

Stops the robot when points are detected in a rectangular region in front of the robot.

Topics
- Subscribes: `/cmd_vel_raw` (geometry_msgs/Twist), `/scan_cloud` (sensor_msgs/PointCloud2)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

Parameters (can be set in a launch file)
- `~front_min_x`, `~front_max_x` (meters)
- `~front_min_y`, `~front_max_y` (meters)
- `~front_min_z`, `~front_max_z` (meters)

Run
- roslaunch safety_planner safety_planner.launch

Notes
- The node forwards raw cmd_vel when no obstacle is present; when an obstacle is detected it publishes zero velocities to stop the robot.
