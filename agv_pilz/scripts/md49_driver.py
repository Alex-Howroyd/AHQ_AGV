#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int64MultiArray
import serial
import struct
import time

port = serial.Serial("/dev/ttyS0", baudrate=38400, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)

class MD49_driver(Node):

    def __init__(self):
        super().__init__('md49_driver')
        self.subscription = self.create_subscription(Int32MultiArray,'/wheels_rpm',self.listener_callback, 10)
        
        self.set_mode(0)
        self.reset_encoders()

    def listener_callback(self, msg):
        right_vel = msg.data[0]
        left_vel = msg.data[1]

        self.set_speeds(left_vel,right_vel)
        self.get_logger().info('right_vel: "%s"' % right_vel)
        self.get_logger().info('left_vel: "%s"' % left_vel)

    def writeBytes(self, bytes):
        port.write(bytes)

    def set_mode(self,mode):
        strMode = struct.pack("ssB",B"\x00",B"\x34",mode)
        self.writeBytes(strMode)

    def reset_encoders(self):
        self.writeBytes(B"\x00\x35")

    def return_mode():
        self.writeBytes(B"\x00\x2B")
        mode = struct.unpack("B",port.read())
        return mode

    def set_speeds(self,speed_1,speed_2):
        strSpeed_1 = struct.pack("ssB",B"\x00",B"\x31",speed_1)
        self.writeBytes(strSpeed_1)
        strSpeed_2 = struct.pack("ssB",B"\x00",B"\x32",speed_2)
        self.writeBytes(strSpeed_2)

    def get_encoder_1(self):
        self.writeBytes(B"\x00\x23")
        encByte_1 = struct.unpack("B",port.read(size=1))
        #if encByte_1[0] != 0:
        self.left_ticks += encByte_1[0]
        return int(self.left_ticks)

    def get_encoder_2(self):
        self.writeBytes(B"\x00\x24")
        encByte_2 = struct.unpack("B",port.read(size=1))
        #if encByte_2[0] != 0:
        self.right_ticks += encByte_2[0]
        return int(self.right_ticks)

def main(args=None):
    rclpy.init(args=args)
    md49_driver = MD49_driver()
    rclpy.spin(md49_driver)
    md49_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
