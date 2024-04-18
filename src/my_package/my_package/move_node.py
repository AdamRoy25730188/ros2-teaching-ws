import rclpy
from geometry_msgs.msg import Twist

class moveNode:

    def __init__(self):
        super().__init__('move_node')
        self.pub = rclpy.Publisher(Twist, '/cmd/vel', 10)

    def run(self):
        while not rclpy.is_shutdown():
            s = Twist()
            s.angular.z = 0.5
            #print('debug: %s' % s)
            self.pub.publish(s)
            rclpy.sleep(.3)


def main(args=None):
    rclpy.init(args=args)
    node = moveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
