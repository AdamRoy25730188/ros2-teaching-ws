#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):

    def __init__(self):
        rclpy.init_node("my_node")
        self.pub = rclpy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        self.timer_ = self.create_timer(0.5, self.run)

    def run(self):
        while not rclpy.is_shutdown():
            s = Twist()
            s.linear.x = 0.5
            s.angular.z = 0.5
            print('debug: %s' % s)
            self.pub.publish(s)

def main(args=None):
    rclpy.init(args=args)
    node = Circle()
    rclpy.spin(node)
    rclpy.shutdown()

'''
myP = MyPublisher()
myP.run()
#rospy.spin()
'''

/#
if __name__ == '__main__':
    main()
