#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Dummy(Node):

    def __init__(self):
        super().__init__('dummy_publisher')
        self.pub_cmd = self.create_publisher(Twist, 'r0/cmd_vel', 10)
        self.sub_laser = self.create_subscription( LaserScan, '/r0/scan_raw',  self.callback_laser, 10)
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.callback_timer)
        
        self.vmax = 0.5
        self.declare_parameter('vmax', self.vmax)

    def callback_laser(self, msg : LaserScan):
        nr_of_ranges = len(msg.ranges)
        if nr_of_ranges == 0: return
        cmd = Twist()
        cmd.linear.x = self.vmax
        if msg.ranges[int(nr_of_ranges/2)] < 1:
            cmd.linear.x = 0.1
        if msg.ranges[int(nr_of_ranges/4)] < 2:
            cmd.angular.z = 0.3
        if msg.ranges[int(nr_of_ranges/4*3)] < 2:
            cmd.angular.z = -0.3
        
        self.pub_cmd.publish(cmd)
        
        
        
    def callback_timer(self):
        vmax = self.get_parameter('vmax').get_parameter_value().double_value
        if self.vmax != vmax :
            self.vmax = vmax 
            self.get_logger().info('update vmax %f!' % self.vmax)



def main(args=None):
    rclpy.init(args=args)

    dummy = Dummy()

    rclpy.spin(dummy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
