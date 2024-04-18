import rclpy
from geometry_msgs.msg import Twist

class Publisher:

    def __init__(self):
        super().__init__('move_node')
        self.pub = rclpy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    def run(self):
        while not rclpy.is_shutdown():
            s = Twist()
            s.angular.z = 0.5
            #print('debug: %s' % s)
            self.pub.publish(s)
            rclpy.sleep(.3)


def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()


#myP = MyPublisher()
#myP.run()
#rospy.spin()

#if __name__ == '__main__':
#    main()
