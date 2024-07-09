import rclpy
from rclpy.node import Node
#from rclpy import Node
from geometry_msgs.msg import Twist

#Main node

class moveNode(Node):

    def __init__(self):
        super().__init__('m_node')        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.05   # m/s
        self.move_cmd.angular.z = 0.5;

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = moveNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print("HyperSharted. %s", e)


if __name__ == '__main__':
    main()
