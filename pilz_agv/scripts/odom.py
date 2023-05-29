#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState

class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.update_odometry)

        self.current_time = self.get_clock().now().to_msg()
        self.last_time = self.get_clock().now().to_msg()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.wheel_radius = 0.0625
        self.wheel_base = 0.33
        self.left_position = 0.0
        self.right_position = 0.0
        self.joint_velocities = [0.0, 0.0]

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def update_odometry(self):
        self.current_time = self.get_clock().now().to_msg()
        self.dt = (self.current_time.sec - self.last_time.sec) + (self.current_time.nanosec - self.last_time.nanosec) / 1e+9

        self.delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * self.dt
        self.delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt

        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        self.last_time = self.current_time

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        self.quat = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = self.quat[0]
        odom.pose.pose.orientation.y = self.quat[1]
        odom.pose.pose.orientation.z = self.quat[2]
        odom.pose.pose.orientation.w = self.quat[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)
    
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "odom"
        transform_stamped.child_frame_id = "base_footprint"
        transform_stamped.transform.translation.x = self.x
        transform_stamped.transform.translation.y = self.y
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = self.quat[0]
        transform_stamped.transform.rotation.y = self.quat[1]
        transform_stamped.transform.rotation.z = self.quat[2]
        transform_stamped.transform.rotation.w = self.quat[3]
        self.tf_broadcaster.sendTransform(transform_stamped)
    
        linear_vel = self.vx
        angular_vel = self.vth
        left_vel = (linear_vel - (angular_vel * self.wheel_base / 2)) / self.wheel_radius
        right_vel = (linear_vel + (angular_vel * self.wheel_base / 2)) / self.wheel_radius
        self.left_position += left_vel * self.dt
        self.right_position += right_vel * self.dt

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['fixed_wheel_right_joint','fixed_wheel_left_joint']
        joint_state.position = [self.right_position, self.left_position]
        joint_state.velocity = [right_vel, left_vel]
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    odom_pub = CmdVelToOdom()
    rclpy.spin(odom_pub)
    odom_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()