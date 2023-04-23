import rclpy
from rclpy.node import Node
from random import random
from nav_msgs.msg import Odometry


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subcriber')
        self.x = None
        self.y = None
        self.z = None
        self.subcriber_ = self.create_subscription(Odometry, 'odom', self.listener_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def timer_callback(self):
        msg = Odometry() 
        with open('odometry_log.txt', 'a') as the_file:
            the_file.write('X: {0:.3f} Y: {1:.3f} Z: {2:.3f}'.format(self.x, self.y, self.z))
        string_to_send = 'X: {0:.3f} Y: {1:.3f} Z: {2:.3f}'.format(self.x, self.y, self.z)
        self.get_logger().info(string_to_send)

    

def main(args = None):
    rclpy.init(args=args)
    odom_subcriber = OdomSubscriber()
    rclpy.spin(odom_subcriber)
    odom_subcriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()