import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist

class moveNode:

    def __init__(self):
        super().__init__('node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd/vel', 10)
        time = 0.1
        self.timer = self.create_timer(time, self.timer_callback)
        s = Twist()
        s.angular.z = 0.5
        s.linear.x = 0.1


    def timer_callback(self):
        self.cmd_vel_pub.publish(self.s)


def main(args=None):
    rclpy.init()
    try:
        node = moveNode()
        rclpy.spin(node)
        rclpy.shutdown()
    except Exception as e:
        print("crapped out", e)

if __name__ == '__main__':
    main()
