# joy2twist
Convert sensor_msgs/Joy to geometry_msgs/Twist.

## Topics
- Subscribes: `joy` (sensor_msgs/Joy)
- Publishes: `cmd_vel_raw` (geometry_msgs/Twist)

## Parameters
- `~linear_axis` (int, default 1)
- `~angular_axis` (int, default 0)
- `~linear_scale` (double, default 1.0)
- `~angular_scale` (double, default 1.0)
- `~deadband` (double, default 0.05)
- `~invert_linear` (bool, default true)
- `~invert_angular` (bool, default false)
- `~publish_zero_on_button` (int, default -1 disabled)

## Launch
- `roslaunch joy2twist joy2twist.launch`

## Notes
- Remap topics as needed; the launch sets `/cmd_vel_raw` by default to chain with downstream safety planner.
