#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist:
    def __init__(self):
        rospy.init_node('joy2twist')
        # Parameters: axis mapping and scale
        self.linear_axis = rospy.get_param('~linear_axis', 1)   # typically left stick vertical
        self.angular_axis = rospy.get_param('~angular_axis', 0)  # typically left stick horizontal
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)
        self.deadband = rospy.get_param('~deadband', 0.05)
        self.invert_linear = rospy.get_param('~invert_linear', True)
        self.invert_angular = rospy.get_param('~invert_angular', False)
        self.publish_zero_on_button = rospy.get_param('~publish_zero_on_button', -1)  # button index; -1 disabled

        self.cmd_pub = rospy.Publisher('cmd_vel_raw', Twist, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_cb)

    def apply_deadband(self, v):
        return 0.0 if abs(v) < self.deadband else v

    def joy_cb(self, msg: Joy):
        twist = Twist()
        # Safety: if button index configured and pressed, force zero output
        if 0 <= self.publish_zero_on_button < len(msg.buttons) and msg.buttons[self.publish_zero_on_button]:
            self.cmd_pub.publish(twist)
            return

        if self.linear_axis < len(msg.axes):
            lin = msg.axes[self.linear_axis]
            if self.invert_linear:
                lin = -lin
            lin = self.apply_deadband(lin) * self.linear_scale
            twist.linear.x = lin
        if self.angular_axis < len(msg.axes):
            ang = msg.axes[self.angular_axis]
            if self.invert_angular:
                ang = -ang
            ang = self.apply_deadband(ang) * self.angular_scale
            twist.angular.z = ang
        self.cmd_pub.publish(twist)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = JoyToTwist()
    node.spin()
