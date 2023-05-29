#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

wheel_diameter = 0.125
gear_ratio = 49
wheel_base = 0.33

old_min = -142.85
old_max = 142.85
new_min = 1
new_max = 255


class WheelsVel(Node):
    def __init__(self):
        super().__init__('wheels_vel')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel',self.cmd_vel_callback,10)
        self.pub = self.create_publisher(Int32MultiArray, '/wheels_rpm', 10)

    def cmd_vel_callback(self, sub_msg):

        # Calculate wheel speed in m/s
        left_wheel_vel = sub_msg.linear.x -((sub_msg.angular.z * wheel_base)/2.0)
        right_wheel_vel = sub_msg.linear.x +((sub_msg.angular.z * wheel_base)/2.0)

        # Convert the required wheel speeds in m/s into rpms
        c = math.pi * wheel_diameter # Wheel circumference in meters
        right_rpm = int((right_wheel_vel / c) * 60.0)
        left_rpm = int((left_wheel_vel / c) * 60.0)

        # Transform rpm range to [1,255]
        new_right_rpm = ((right_rpm - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        new_left_rpm = ((left_rpm - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

        msg_pub = Int32MultiArray()
        msg_pub.data = [int(new_right_rpm),int(new_left_rpm)]
        self.pub.publish(msg_pub)

        self.get_logger().info('Wheels_vel: "%f"' % new_right_rpm)

def main(args=None):
    rclpy.init(args=args)
    wheels_vel = WheelsVel()
    rclpy.spin(wheels_vel)
    wheels_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()