#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomPub(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.R = 0.0625        # Wheel Radius
        self.N = 588            # Number of Ticks Per revolution FAAAAAAAAAAAAAAAAAALTA
        self.L = 0.33          # Distance between left and right wheels

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx =  0.0
        self.vy =  0.0
        self.vth =  0.0

        self.dc = 0.0
        self.dr = 0.0
        self.dl = 0.0
        self.dt = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.dtheta = 0.0

        self.ex_Left_Ticks = 0
        self.ex_Right_Ticks = 0
        
        self.encoder_ticks_subscription = self.create_subscription(Int64MultiArray,'/encoders_ticks',self.encoder_ticks,10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.l_t = self.get_clock().now().to_msg()
        self.last_time = self.l_t.sec + (self.l_t.nanosec/1e+9)

        self.broadcaster = TransformBroadcaster(self, 10)

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        #self.joint_state = JointState()

    def encoder_ticks(self, sub_msg):

        self.Left_Ticks = sub_msg.data[0]
        self.Right_Ticks = sub_msg.data[1]

        self.Odom_calculation()
    
    def Odom_calculation(self):

        self.c_t = self.get_clock().now().to_msg()
        self.current_time = self.c_t.sec + (self.c_t.nanosec/1e+9)
    
        self.L_delta_Tick = self.Left_Ticks - self.ex_Left_Ticks
        self.get_logger().info('L_delta_Tick: "%s"' % self.L_delta_Tick)
        self.R_delta_Tick = self.Right_Ticks - self.ex_Right_Ticks
        self.get_logger().info('R_delta_Tick: "%s"' % self.R_delta_Tick)

        self.dt = (self.current_time - self.last_time)
        #self.get_logger().info('dt: "%s"' % self.dt)
        self.dl = 2 * math.pi * self.R * self.L_delta_Tick / self.N
        self.dr = 2 * math.pi * self.R * self.R_delta_Tick / self.N
        #self.get_logger().info('dl: "%s"' % self.dl)
        #self.get_logger().info('dr: "%s"' % self.dr)
        self.dc = (self.dl + self.dr) / 2
        self.dtheta = (self.dr - self.dl) / self.L
        
        if self.dr == self.dl:
            self.dx = self.dr*math.cos(self.th)
            self.dy = self.dr*math.sin(self.th)
        else:
            self.radius = self.dc/self.dtheta
            self.iccX = self.x - self.radius*math.sin(self.th)
            self.iccY = self.y + self.radius*math.cos(self.th)
            self.dx = math.cos(self.dtheta) * (self.x - self.iccX) - math.sin(self.dtheta) * (self.y - self.iccY) + self.iccX - self.x
            self.dy = math.sin(self.dtheta) * (self.x - self.iccX) - math.cos(self.dtheta) * (self.y - self.iccY) + self.iccY - self.y
        
        self.x += self.dx  # delta_x
        self.y += self.dy  #delta_y
        self.th = (self.th + self.dtheta) % (2 * math.pi) #delta_th

        #self.get_logger().info('x: "%s"' % self.x)
        self.get_logger().info('y: "%s"' % self.y)
        #self.get_logger().info('th: "%s"' % self.th)

        self.odom.pose.pose.position.x = self.x/1000
        self.odom.pose.pose.position.y = self.y/1000
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0/1000

        self.vx = self.dx/self.dt
        self.vy = self.dy/self.dt
        self.vth = self.dtheta/self.dt
        
        self.odom.twist.twist.linear.x = self.vx/1000
        self.odom.twist.twist.linear.y = self.vy/1000
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = self.vth/1000

        self.odom_pub.publish(self.odom)

        self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_trans.transform.translation.z = self.odom.pose.pose.position.z
        self.odom_trans.transform.rotation.x = self.odom.pose.pose.orientation.x
        self.odom_trans.transform.rotation.y = self.odom.pose.pose.orientation.y
        self.odom_trans.transform.rotation.z = self.odom.pose.pose.orientation.z
        self.odom_trans.transform.rotation.w = self.odom.pose.pose.orientation.w

        self.broadcaster.sendTransform(self.odom_trans)

        self.ex_Left_Ticks = self.Left_Ticks
        self.ex_Right_Ticks = self.Right_Ticks
        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    odom_pub = OdomPub()
    rclpy.spin(odom_pub)
    odom_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()