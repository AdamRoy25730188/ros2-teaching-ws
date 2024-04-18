import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

'''
An attempt to find a way to integrate lidar to find depth, lacked time to impliment
'''

class Roamer(Node):

    def __init__(self):
        super().__init__('roamer')
        self.laser_sub = self.create_subscription(
            LaserScan,"/scan",
            self.callback, 1)
        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel', 1)

    def callback(self, data):
        min_range = data.range_max
        print(len(data.ranges))
        for v in data.ranges:
            if v < min_range:
                min_range = v
        print(min_range)
        twist = Twist()
        if min_range < 1.0:
            twist.angular.z = 1.0
        else:
            twist.linear.x = 0.2
        self.twist_pub.publish(twist)
            
        

try:
    rclpy.init()
    node = Roamer()
    rclpy.spin(node)
finally:
    try:
        node.destroy_node()
    finally: 
        rclpy.shutdown()