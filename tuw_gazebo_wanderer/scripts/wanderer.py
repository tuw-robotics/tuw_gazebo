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
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback_timer)
        

    def callback_laser(self, msg : LaserScan):
        
        self.get_logger().info('I heard %f' % msg.ranges[0])
        if len(msg.ranges) == 0: return
        
        msg = Twist()
        msg.linear.x = 0.1
        self.pub_cmd.publish(msg)
        
        
        
    def callback_timer(self):
        msg = Twist()
        #msg.linear.x = 0.1
        #self.publisher_.publish(msg)


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
